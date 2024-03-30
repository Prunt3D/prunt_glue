with Motion_Planner.Planner;
with Stepgen.Stepgen;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Physical_Types;        use Physical_Types;

private generic
package Prunt_Glue.Glue.Gcode_Handler is

   procedure Try_Set_File (Path : String; Succeeded : out Boolean);
   procedure Try_Queue_Command (Command : String; Succeeded : out Boolean);

   procedure Finished_Block
     (Data : Flush_Extra_Data; First_Segment_Accel_Distance : Physical_Types.Length; Hit_During_Accel : Boolean);

   task Runner is
      entry Start;
   end Runner;

private

   protected Gcode_Queue is
      procedure Try_Set_File (In_File : String; Succeeded : out Boolean);
      procedure Clear_File;
      function Get_File return String;
      procedure Try_Set_Command (In_Command : String; Succeeded : out Boolean);
      procedure Clear_Command;
      function Get_Command return String;
   private
      File    : Unbounded_String := To_Unbounded_String ("");
      Command : Unbounded_String := To_Unbounded_String ("");
   end Gcode_Queue;

   protected Finished_Block_Queue is
      procedure Push
        (In_Data                         : Flush_Extra_Data;
         In_First_Segment_Accel_Distance : Physical_Types.Length;
         In_Hit_During_Accel             : Boolean);
      entry Pop
        (Out_Data                         : out Flush_Extra_Data;
         Out_First_Segment_Accel_Distance : out Physical_Types.Length;
         Out_Hit_During_Accel             : out Boolean);
   private
      Has_Item                     : Boolean := False;
      Data                         : Flush_Extra_Data;
      First_Segment_Accel_Distance : Physical_Types.Length;
      Hit_During_Accel             : Boolean;
   end Finished_Block_Queue;

end Prunt_Glue.Glue.Gcode_Handler;
