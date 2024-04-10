with Motion_Planner;
with Physical_Types;        use Physical_Types;
with System.Multiprocessors;
with Config.Config;
with Stepgen.Stepgen;
with Motion_Planner.Planner;
with GUI.GUI;
with Ada.Containers;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;

generic
   type Low_Level_Time_Type is mod <>;
   --  Native type of the clock source.
   --
   --  Warning: The stepgen implementation currently assumes that this value will never overflow, which is fine for a
   --  MHz range 64-bit clock, but not fine for 32-bit or absurdly fast clocks.
   with function Time_To_Low_Level (T : Time) return Low_Level_Time_Type;
   with function Low_Level_To_Time (T : Low_Level_Time_Type) return Time;
   with function Get_Time return Low_Level_Time_Type;

   type Stepper_Name is (<>);
   --  'Image of each value of this type will be shown in the GUI. The names should correspond to names on the board.
   with procedure Set_Stepper_Pin_State (Stepper : Stepper_Name; Pin : Stepper_Output_Pins; State : Pin_State);
   with function Get_Stepper_Pin_State (Stepper : Stepper_Name; Pin : Stepper_Input_Pins) return Pin_State;

   type Heater_Name is (<>);
   --  'Image of each value of this type will be shown in the GUI. The names should correspond to names on the board.
   with procedure Set_Heater_PWM (Heater : Heater_Name; PWM : PWM_Scale);

   type Thermistor_Name is (<>);
   --  'Image of each value of this type will be shown in the GUI. The names should correspond to names on the board.
   with function Get_Thermistor_Voltage (Thermistor : Thermistor_Name) return Voltage;

   type Fan_Name is (<>);
   --  'Image of each value of this type will be shown in the GUI. The names should correspond to names on the board.
   with procedure Set_Fan_PWM (Fan : Fan_Name; PWM : PWM_Scale);
   with procedure Set_Fan_Voltage (Fan : Fan_Name; Volts : Voltage);
   with function Get_Fan_Frequency (Fan : Fan_Name) return Frequency;

   type Input_Switch_Name is (<>);
   --  'Image of each value of this type will be shown in the GUI. The names should correspond to names on the board.
   with function Get_Input_Switch_State (Switch : Input_Switch_Name) return Pin_State;

   Stepgen_Preprocessor_CPU : System.Multiprocessors.CPU_Range;
   --  CPU for step generator preprocessor task. This task requires an isolated CPU where it is the only task.
   Stepgen_Pulse_Generator_CPU : System.Multiprocessors.CPU_Range;
   --  CPU for step generator pulse generator task. This task requires an isolated CPU where it is the only task.

   Config_Path : String;
   --  Path of the printer configuration file.

   Interpolation_Time : Low_Level_Time_Type;
   --  The time between points that are passed to the step pulse generator. This value should be as low as possible
   --  while avoiding the point queue running dry. In a future version the fullness of the of the queue will be
   --  presented in the GUI to make tuning this value easier.

   --  TODO: Replace the following parameters with a single Target_Memory_Usage parameter.

   Max_Planner_Block_Corners : Motion_Planner.Max_Corners_Type := 3_000;
   --  Number of corners to be planned in a single block. Increasing this value will minimise the number of complete
   --  stops required at the cost of using more memory. Increasing this value too far will cause the printer to pause
   --  after each block while the next is loaded.

   Planner_Input_Queue_Length : Ada.Containers.Count_Type := 1_000;
   Planner_Output_Queue_Length : Ada.Containers.Count_Type := 1;
   --  Maximum number of items in the planner input and output queues. Increasing these values uses more memory but
   --  provides a larger buffer that may increase throughput. There are no serious consequences to these queues
   --  running dry, the printer may just pause more often as it waits for new blocks.

   with procedure Waiting_For_Time (T : Low_Level_Time_Type) is null;
   --  When the step generator is waiting in a loop for a specific time, this procedure will be called. This procedure
   --  is only intended for use with simulation. Do not use this procedure to do other things in the stepgen task.

   Ignore_Empty_Queue : Boolean := False;
   --  If set to True, the step generator will not raise an exception when the queue is empty and will not wait for
   --  the queue to be filled before beginning execution. This is only meant for use in simulation.
package Prunt_Glue.Glue is

   procedure Run;

private

   Loop_Interpolation_Time_Multiplier : constant := 32;

   procedure Helper_Lock_Memory with
     Import => True, Convention => C, External_Name => "prunt_glue_helper_lock_memory";

   type Flush_Extra_Data is record
      Is_Homing_Move : Boolean           := False;
      Home_Switch    : Input_Switch_Name := Input_Switch_Name'First;
      Hit_On_State   : Pin_State         := High_State;
   end record;

   function Is_Homing_Move (Data : Flush_Extra_Data) return Boolean;
   function Is_Home_Switch_Hit (Data : Flush_Extra_Data) return Boolean;

   package My_Planner is new Motion_Planner.Planner
     (Flush_Extra_Data_Type        => Flush_Extra_Data,
      Flush_Extra_Data_Default     => (others => <>),
      Initial_Position             => [others => 0.0 * mm],
      Max_Corners                  => Max_Planner_Block_Corners,
      Input_Queue_Length           => Planner_Input_Queue_Length,
      Output_Queue_Length          => Planner_Output_Queue_Length,
      Is_Homing_Move               => Is_Homing_Move,
      Home_Move_Minimum_Coast_Time =>
        1.03 * Low_Level_To_Time (Interpolation_Time * (1 + Loop_Interpolation_Time_Multiplier)));

   package My_Config is new Config.Config
     (Stepper_Name      => Stepper_Name,
      Heater_Name       => Heater_Name,
      Thermistor_Name   => Thermistor_Name,
      Fan_Name          => Fan_Name,
      Input_Switch_Name => Input_Switch_Name,
      Config_Path       => Config_Path);

   type Stepper_Position is array (Stepper_Name) of Stepgen.Step_Count;

   type Stepper_Pos_Data is array (Axis_Name, Stepper_Name) of Physical_Types.Length;

   function Position_To_Stepper_Position (Pos : Position; Data : Stepper_Pos_Data) return Stepper_Position;
   function Stepper_Position_To_Position (Pos : Stepper_Position; Data : Stepper_Pos_Data) return Position;

   type Stepper_Output_Data is record
      Current_Step_State : Pin_State;
   end record;

   procedure Do_Step (Stepper : Stepper_Name; Data : in out Stepper_Output_Data);
   procedure Set_Direction (Stepper : Stepper_Name; Dir : Stepgen.Direction; Data : in out Stepper_Output_Data);

   procedure Finished_Block
     (Data : Flush_Extra_Data; First_Segment_Accel_Distance : Physical_Types.Length; Hit_During_Accel : Boolean);

   function Get_Status_Message return String;

   function Get_Position return Position;

   procedure Submit_Gcode_Command (Command : String; Succeeded : out Boolean);
   procedure Submit_Gcode_File (Path : String; Succeeded : out Boolean);

   package My_Stepgen is new Stepgen.Stepgen
     (Low_Level_Time_Type          => Low_Level_Time_Type,
      Time_To_Low_Level            => Time_To_Low_Level,
      Low_Level_To_Time            => Low_Level_To_Time,
      Get_Time                     => Get_Time,
      Planner                      => My_Planner,
      Is_Homing_Move               => Is_Homing_Move,
      Is_Home_Switch_Hit           => Is_Home_Switch_Hit,
      Stepper_Name                 => Stepper_Name,
      Stepper_Position             => Stepper_Position,
      Stepper_Pos_Data             => Stepper_Pos_Data,
      Position_To_Stepper_Position => Position_To_Stepper_Position,
      Stepper_Position_To_Position => Stepper_Position_To_Position,
      Stepper_Output_Data          => Stepper_Output_Data,
      Do_Step                      => Do_Step,
      Set_Direction                => Set_Direction,
      Finished_Block               => Finished_Block,
      Interpolation_Time           => Interpolation_Time,
      Loop_Interpolation_Time      => Interpolation_Time * Loop_Interpolation_Time_Multiplier,
      Initial_Position             => [others => 0.0 * mm],
      Preprocessor_CPU             => Stepgen_Preprocessor_CPU,
      Runner_CPU                   => Stepgen_Pulse_Generator_CPU,
      Waiting_For_Time             => Waiting_For_Time,
      Ignore_Empty_Queue           => Ignore_Empty_Queue);

   package My_GUI is new GUI.GUI
     (My_Config            => My_Config,
      Get_Status_Message   => Get_Status_Message,
      Get_Position         => Get_Position,
      Submit_Gcode_Command => Submit_Gcode_Command,
      Submit_Gcode_File    => Submit_Gcode_File);

   protected Status_Message is
      procedure Set (S : String);
      function Get return String;
   private
      Local : Unbounded_String := To_Unbounded_String ("");
   end Status_Message;

end Prunt_Glue.Glue;
