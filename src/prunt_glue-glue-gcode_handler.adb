-----------------------------------------------------------------------------
--                                                                         --
--                   Part of the Prunt Motion Controller                   --
--                                                                         --
--            Copyright (C) 2024 Liam Powell (liam@prunt3d.com)            --
--                                                                         --
--  This program is free software: you can redistribute it and/or modify   --
--  it under the terms of the GNU General Public License as published by   --
--  the Free Software Foundation, either version 3 of the License, or      --
--  (at your option) any later version.                                    --
--                                                                         --
--  This program is distributed in the hope that it will be useful,        --
--  but WITHOUT ANY WARRANTY; without even the implied warranty of         --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          --
--  GNU General Public License for more details.                           --
--                                                                         --
--  You should have received a copy of the GNU General Public License      --
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.  --
--                                                                         --
-----------------------------------------------------------------------------

with Gcode_Parser; use Gcode_Parser;
with Ada.Text_IO;  use Ada.Text_IO;
with Ada.Exceptions;

package body Prunt_Glue.Glue.Gcode_Handler is

   procedure Set_Status_Message (S : String) is
   begin
      null;
      --  TODO
   end Set_Status_Message;

   procedure Try_Set_File (Path : String; Succeeded : out Boolean) is
   begin
      Gcode_Queue.Try_Set_File (Path, Succeeded);
   end Try_Set_File;

   procedure Try_Queue_Command (Command : String; Succeeded : out Boolean) is
   begin
      Gcode_Queue.Try_Set_Command (Command, Succeeded);
   end Try_Queue_Command;

   task body Runner is
      --  TODO: Track down GNAT bug that stops [others => 0.0 * mm] from working specifically in this file.
      Zero_Pos : constant Position := [X_Axis => 0.0 * mm, Y_Axis => 0.0 * mm, Z_Axis => 0.0 * mm, E_Axis => 0.0 * mm];
      Zero_Pos_Offset : constant Position_Offset :=
        [X_Axis => 0.0 * mm, Y_Axis => 0.0 * mm, Z_Axis => 0.0 * mm, E_Axis => 0.0 * mm];

      Parser_Context : Gcode_Parser.Context := Make_Context (Zero_Pos, 100.0 * mm / s);

      Is_Homed : array (Axis_Name) of Boolean := [others => False];

      Kinematics_Params        : My_Config.Kinematics_Parameters;
      Axial_Homing_Params      : array (Axis_Name) of My_Config.Homing_Parameters;
      Switchwise_Switch_Params : array (Input_Switch_Name) of My_Config.Input_Switch_Parameters;

      Command_Constraint_Error : exception;

      procedure Check_Bounds (Pos : Position) is
      begin
         for I in Axis_Name loop
            if Pos (I) < Kinematics_Params.Lower_Pos_Limit (I) or Pos (I) > Kinematics_Params.Upper_Pos_Limit (I)
            then
               raise Command_Constraint_Error
                 with "Position is out of bounds (" & I'Image & " = " & Pos (I)'Image & "):";
            end if;
         end loop;
      end Check_Bounds;

      procedure Enforce_Feedrate_Limits (Offset : Position_Offset; Feedrate : in out Velocity) is
         Has_XYZ : Boolean := [Offset with delta E_Axis => 0.0 * mm] /= Zero_Pos_Offset;
      begin
         if Kinematics_Params.Ignore_E_Feedrate_In_XYZE_Moves and Has_XYZ then
            Feedrate := Feedrate * abs Offset / abs [Offset with delta E_Axis => 0.0 * mm];
         end if;

         if Feedrate > Kinematics_Params.Max_Feedrate then
            Feedrate := Kinematics_Params.Max_Feedrate;
         end if;

         for I in Axis_Name loop
            if abs Offset (I) > 0.0 * mm then
               Feedrate :=
                 Velocity'Min (Feedrate, Kinematics_Params.Max_Axial_Velocities (I) * abs Offset / abs Offset (I));
            end if;
         end loop;
      end Enforce_Feedrate_Limits;

      procedure Double_Tap_Home_Axis (Axis : Axis_Name; Pos_After : in out Position) is
         Switch          : Input_Switch_Name := Axial_Homing_Params (Axis).Switch;
         Hit_State       : Pin_State         :=
           (if Switchwise_Switch_Params (Axial_Homing_Params (Axis).Switch).Hit_On_High then High_State
            else Low_State);
         First_Offset : Position_Offset :=
           [Zero_Pos_Offset with delta Axis => Axial_Homing_Params (Axis).First_Move_Distance];
         Second_Offset : Position_Offset :=
           [Zero_Pos_Offset with delta Axis => Axial_Homing_Params (Axis).Second_Move_Distance];
         First_Feedrate  : Velocity          := Kinematics_Params.Max_Feedrate;
         Second_Feedrate : Velocity          := Kinematics_Params.Max_Feedrate;
      begin
         Enforce_Feedrate_Limits (First_Offset, First_Feedrate);
         Enforce_Feedrate_Limits (Second_Offset, Second_Feedrate);

         My_Planner.Enqueue
           ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
             Flush_Extra_Data => (others => <>),
             Reset_Pos        => Zero_Pos));

         if Get_Input_Switch_State (Switch) = Hit_State then
            My_Planner.Enqueue
              ((Kind => My_Planner.Move_Kind, Pos => Zero_Pos - First_Offset, Feedrate => First_Feedrate));
            My_Planner.Enqueue
              ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
                Flush_Extra_Data => (others => <>),
                Reset_Pos        => Zero_Pos));
         end if;

         My_Planner.Enqueue
           ((Kind => My_Planner.Move_Kind, Pos => Zero_Pos + First_Offset, Feedrate => First_Feedrate));
         My_Planner.Enqueue
           ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
             Flush_Extra_Data => (Is_Homing_Move => True, Home_Switch => Switch, Hit_On_State => Hit_State),
             Reset_Pos        => Zero_Pos));
         My_Planner.Enqueue
           ((Kind => My_Planner.Move_Kind, Pos => Zero_Pos - First_Offset - First_Offset, Feedrate => First_Feedrate));
         My_Planner.Enqueue
           ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
             Flush_Extra_Data => (others => <>),
             Reset_Pos        => Zero_Pos));

         declare
            Data             : Flush_Extra_Data;
            First_Seg_Accel  : Physical_Types.Length;
            Hit_During_Accel : Boolean;
         begin
            Finished_Block_Queue.Pop (Data, First_Seg_Accel, Hit_During_Accel);
            --  We do not care what about happened during the homing move at this point as long as
            --  the switch is not hit.
         end;

         --  TODO: This is not a good solution. We should instead tag the back-off move so it goes in to the finished
         --  block queue, then we can wait for that here instead of using a delay.
         delay 1.0;
         My_Stepgen.Wait_Until_Idle;

         if Get_Input_Switch_State (Switch) = Hit_State then
            raise Command_Constraint_Error with "Homing switch still hit after backing off after first hit.";
         end if;

         My_Planner.Enqueue
           ((Kind => My_Planner.Move_Kind, Pos => Zero_Pos + Second_Offset, Feedrate => Second_Feedrate));
         My_Planner.Enqueue
           ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
             Flush_Extra_Data => (Is_Homing_Move => True, Home_Switch => Switch, Hit_On_State => Hit_State),
             Reset_Pos        => Zero_Pos));

         declare
            Data             : Flush_Extra_Data;
            First_Seg_Accel  : Physical_Types.Length;
            Hit_During_Accel : Boolean;
         begin
            Finished_Block_Queue.Pop (Data, First_Seg_Accel, Hit_During_Accel);

            if Axial_Homing_Params (Axis).Second_Move_Distance < 0.0 * mm then
               First_Seg_Accel := -First_Seg_Accel;
            end if;

            Pos_After (Axis) :=
              Axial_Homing_Params (Axis).Switch_Position + Axial_Homing_Params (Axis).Second_Move_Distance -
              First_Seg_Accel;

            if Hit_During_Accel then
               raise Command_Constraint_Error
                 with "Homing switch hit at physically impossible position during second move.";
            end if;
         end;

         My_Planner.Enqueue
           ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
             Flush_Extra_Data => (others => <>),
             Reset_Pos        => Pos_After));

         declare
            Pos_Before_Final_Move : Position := Pos_After;
            Final_Feedrate        : Velocity := Kinematics_Params.Max_Feedrate;
         begin
            if Pos_After (Axis) < Kinematics_Params.Lower_Pos_Limit (Axis) then
               Pos_After (Axis) := Kinematics_Params.Lower_Pos_Limit (Axis);
            elsif Pos_After (Axis) > Kinematics_Params.Upper_Pos_Limit (Axis) then
               Pos_After (Axis) := Kinematics_Params.Upper_Pos_Limit (Axis);
            end if;

            Enforce_Feedrate_Limits (Pos_Before_Final_Move - Pos_After, Final_Feedrate);

            My_Planner.Enqueue ((Kind => My_Planner.Move_Kind, Pos => Pos_After, Feedrate => Final_Feedrate));
            My_Planner.Enqueue
              ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
                Flush_Extra_Data => (others => <>),
                Reset_Pos        => Pos_After));
            Gcode_Parser.Reset_Position (Parser_Context, Pos_After);
         end;
      end Double_Tap_Home_Axis;

      procedure Run_Command (Command : Gcode_Parser.Command) is
      begin
         case Command.Kind is
            when None_Kind =>
               null;
            when Move_Kind =>
               Check_Bounds (Command.Pos);

               for I in Axis_Name loop
                  if not Is_Homed (I) then
                     raise Command_Constraint_Error
                       with "Must home all axes before moving. Axis " & I'Image & " is not homed.";
                  end if;
               end loop;

               declare
                  Feedrate : Velocity := Command.Feedrate;
               begin
                  Enforce_Feedrate_Limits (Command.Old_Pos - Command.Pos, Feedrate);
                  My_Planner.Enqueue ((Kind => My_Planner.Move_Kind, Pos => Command.Pos, Feedrate => Feedrate));
               end;
            when Reset_Position_Kind =>
               Check_Bounds (Command.New_Pos);

               My_Planner.Enqueue
                 ((Kind            => My_Planner.Flush_And_Reset_Position_Kind,
                   Reset_Pos       => Command.New_Pos,
                  Flush_Extra_Data => (others => <>)));
                  Gcode_Parser.Reset_Position (Parser_Context, Command.New_Pos);
            when Home_Kind =>
               declare
                  Pos_After    : Position                       := Command.Pos_Before;
                  Homing_Order : array (Axis_Name) of Axis_Name := [E_Axis, Z_Axis, X_Axis, Y_Axis];
               begin
                  for Axis of Homing_Order loop
                     if Command.Axes (Axis) then
                        case Axial_Homing_Params (Axis).Kind is
                           when My_Config.Set_To_Value_Kind =>
                              Pos_After (Axis) := Axial_Homing_Params (Axis).Value;
                              My_Planner.Enqueue
                                ((Kind             => My_Planner.Flush_And_Reset_Position_Kind,
                                  Reset_Pos        => Pos_After,
                                  Flush_Extra_Data => (others => <>)));
                              Gcode_Parser.Reset_Position (Parser_Context, Pos_After);
                           when My_Config.Double_Tap_Kind =>
                              Double_Tap_Home_Axis (Axis, Pos_After);
                        end case;
                        Is_Homed (Axis) := True;
                     end if;
                  end loop;
               exception
                  when E : others =>
                     --  If homing fails for any reason then all axes become unhomed.
                     Is_Homed := [others => False];
                     raise;
               end;

            when others =>
               raise Constraint_Error with "Command not implemented.";
         end case;
      end Run_Command;
   begin
      accept Start do
         My_Config.Config_File.Read (Kinematics_Params);

         for I in Axis_Name loop
            My_Config.Config_File.Read (Axial_Homing_Params (I), I);
            case Axial_Homing_Params (I).Kind is
               when My_Config.Double_Tap_Kind =>
                  if Axial_Homing_Params (I).First_Move_Distance = 0.0 * mm or
                    Axial_Homing_Params (I).Second_Move_Distance = 0.0 * mm
                  then
                     null;
                     --  TODO: Handle this.
                  end if;

                  if Axial_Homing_Params (I).First_Move_Distance / Axial_Homing_Params (I).Second_Move_Distance <
                    0.0
                  then
                     null;
                     --  TODO: Handle this.
                  end if;

                  if abs Axial_Homing_Params (I).First_Move_Distance <
                    abs Axial_Homing_Params (I).Second_Move_Distance
                  then
                     null;
                     --  TODO: Handle this (larger distance for second move is nonsensical as accuracy will be lower).
                  end if;
               when My_Config.Set_To_Value_Kind =>
                  null;
            end case;
         end loop;
      end Start;

      for I in Input_Switch_Name loop
         My_Config.Config_File.Read (Switchwise_Switch_Params (I), I);
         --  TODO: Check relevant switches are enabled.
      end loop;

      loop
         delay 0.1;

         if Gcode_Queue.Get_Command /= "" then
            declare
               Command           : Gcode_Parser.Command := (others => <>);
               Line              : String               := Gcode_Queue.Get_Command;
               Command_Succeeded : Boolean;
            begin
               Parse_Line (Parser_Context, Line, Command);
               Run_Command (Command);
               My_Planner.Enqueue ((Kind => My_Planner.Flush_Kind, Flush_Extra_Data => (others => <>)));
            exception
               when E : Bad_Line =>
                  Set_Status_Message
                    ("Error parsing manual command (" & Line & "): " & Ada.Exceptions.Exception_Information (E));
            end;

            Gcode_Queue.Clear_Command;
         elsif Gcode_Queue.Get_File /= "" then
            declare
               File : File_Type;
            begin
               Open (File, In_File, Gcode_Queue.Get_File);

               declare
                  type File_Line_Count is range 1 .. 2**63 - 1;
                  Command           : Gcode_Parser.Command := (others => <>);
                  Current_Line      : File_Line_Count      := 1;
                  Command_Succeeded : Boolean              := True;
               begin
                  while Command_Succeeded and not End_Of_File (File) loop
                     declare
                        Line : String := Get_Line (File);
                     begin
                        Parse_Line (Parser_Context, Line, Command);
                        Run_Command (Command);
                     exception
                        when E : Bad_Line =>
                           Set_Status_Message
                             ("Error parsing line in file " & Gcode_Queue.Get_File & " on line " & Current_Line'Image &
                              " (" & Line & "): " & Ada.Exceptions.Exception_Information (E));
                     end;
                  end loop;
               end;

               My_Planner.Enqueue ((Kind => My_Planner.Flush_Kind, Flush_Extra_Data => (others => <>)));

               Close (File);
            exception
               --  TODO: Check what exceptions can actually come from file IO.
               when E : others =>
                  Set_Status_Message
                    ("IO error when processing file " & Gcode_Queue.Get_File & ": " &
                     Ada.Exceptions.Exception_Information (E));
            end;

            Gcode_Queue.Clear_File;
         end if;
      end loop;
   exception
      when E : others =>
         Put_Line ("Error in Prunt_Glue.Gcode_Handler: ");
         Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

   protected body Gcode_Queue is
      procedure Try_Set_File (In_File : String; Succeeded : out Boolean) is
      begin
         if File /= "" then
            Succeeded := False;
            --  Other file already running.
         else
            Set_Unbounded_String (File, In_File);
            Succeeded := True;
         end if;
      end Try_Set_File;

      procedure Clear_File is
      begin
         Set_Unbounded_String (File, "");
      end Clear_File;

      function Get_File return String is
      begin
         return To_String (File);
      end Get_File;

      procedure Try_Set_Command (In_Command : String; Succeeded : out Boolean) is
      begin
         if Command /= "" then
            Succeeded := False;
            --  Other command already running.
         elsif File /= "" then
            Succeeded := False;
            --  File already running. Commands can not be run at same time.
         else
            Set_Unbounded_String (Command, In_Command);
            Succeeded := True;
         end if;
      end Try_Set_Command;

      procedure Clear_Command is
      begin
         Set_Unbounded_String (Command, "");
      end Clear_Command;

      function Get_Command return String is
      begin
         return To_String (Command);
      end Get_Command;
   end Gcode_Queue;

   procedure Finished_Block
     (Data : Flush_Extra_Data; First_Segment_Accel_Distance : Physical_Types.Length; Hit_During_Accel : Boolean)
   is
   begin
      if Data.Is_Homing_Move then
         Finished_Block_Queue.Push (Data, First_Segment_Accel_Distance, Hit_During_Accel);
      end if;
   end Finished_Block;

   protected body Finished_Block_Queue is
      procedure Push
        (In_Data                         : Flush_Extra_Data;
         In_First_Segment_Accel_Distance : Physical_Types.Length;
         In_Hit_During_Accel             : Boolean)
      is
      begin
         if Has_Item then
            raise Constraint_Error with "There should not be more than one homing move in the pipeline.";
         end if;

         Data                         := In_Data;
         First_Segment_Accel_Distance := In_First_Segment_Accel_Distance;
         Hit_During_Accel             := In_Hit_During_Accel;
         Has_Item                     := True;
      end Push;

      entry Pop
        (Out_Data                         : out Flush_Extra_Data;
         Out_First_Segment_Accel_Distance : out Physical_Types.Length;
         Out_Hit_During_Accel             : out Boolean)
        when Has_Item
      is
      begin
         Out_Data                         := Data;
         Out_First_Segment_Accel_Distance := First_Segment_Accel_Distance;
         Out_Hit_During_Accel             := Hit_During_Accel;
         Has_Item                         := False;
      end Pop;
   end Finished_Block_Queue;

end Prunt_Glue.Glue.Gcode_Handler;
