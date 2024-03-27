with Motion_Planner;
with Physical_Types; use Physical_Types;
with System.Multiprocessors;
with Config.Config;
with Stepgen.Stepgen;
with Motion_Planner.Planner;
with GUI.GUI;
with Ada.Containers;

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
   with procedure Toggle_Stepper_Pin_State (Stepper : Stepper_Name; Pin : Stepper_Output_Pins);
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

   Planner_CPU : System.Multiprocessors.CPU_Range;
   --  CPU for motion planner task. This task does not require an isolated CPU.
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
package Prunt_Glue.Glue is

   procedure Run;

private

   package My_Planner is new Motion_Planner.Planner
     (Flush_Extra_Data_Type    => Boolean,
      Flush_Extra_Data_Default => False,
      Initial_Position         => [others => 0.0 * mm],
      Max_Corners              => Max_Planner_Block_Corners,
      Input_Queue_Length       => Planner_Input_Queue_Length,
      Output_Queue_Length      => Planner_Output_Queue_Length);

   package My_Config is new Config.Config
     (Stepper_Name      => Stepper_Name,
      Heater_Name       => Heater_Name,
      Thermistor_Name   => Thermistor_Name,
      Fan_Name          => Fan_Name,
      Input_Switch_Name => Input_Switch_Name,
      Config_Path       => Config_Path);

   package My_GUI is new GUI.GUI (My_Config => My_Config);

   --  package My_Stepgen is new Stepgen.Stepgen
   --    (Low_Level_To_Time            => Low_Level_Time_Type,
   --     Time_To_Low_Level            => Time_To_Low_Level,
   --     Low_Level_To_Time            => Low_Level_To_Time,
   --     Get_Time                     => Get_Time,
   --     Planner                      => My_Planner,
   --     Is_Homing_Move               => X,
   --     Is_Home_Switch_Hit           => X,
   --     Stepper_Name                 => Stepper_Name,
   --     Stepper_Position             => X,
   --     Position_To_Stepper_Position => X,
   --     Stepper_Position_To_Position => X,
   --     Do_Step                      => X,
   --     Set_Direction                => X,
   --     Finished_Block               => X,
   --     Interpolation_Time           => Interpolation_Time,
   --     Initial_Position             => [others => 0.0 * mm]);

end Prunt_Glue.Glue;
