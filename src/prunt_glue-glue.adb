with Ada.Exceptions;

package body Prunt_Glue.Glue is

   function Is_Homing_Move (Data : Flush_Extra_Data) return Boolean is
   begin
      return Data.Is_Homing_Move;
   end Is_Homing_Move;

   function Is_Home_Switch_Hit (Data : Flush_Extra_Data) return Boolean is
   begin
      return Get_Input_Switch_State (Data.Home_Switch) = Data.Hit_On_State;
   end Is_Home_Switch_Hit;

   function Position_To_Stepper_Position (Pos : Position; Data : Stepper_Pos_Data) return Stepper_Position is
      Ret : Stepper_Position := [others => 0];
      use type Stepgen.Step_Count;
   begin
      for S in Stepper_Name loop
         for A in Axis_Name loop
            Ret (S) := Ret (S) + Stepgen.Step_Count (Dimensionless'Floor (Pos (A) / Data (A, S)));
         end loop;
      end loop;

      return Ret;
   end Position_To_Stepper_Position;

   function Stepper_Position_To_Position (Pos : Stepper_Position; Data : Stepper_Pos_Data) return Position is
      Ret : Position;
   begin
      for A in Axis_Name loop
         Ret (A) := 0.0 * mm;
      end loop;

      for S in Stepper_Name loop
         for A in Axis_Name loop
            Ret (A) := Ret (A) + Dimensionless (Pos (S)) * Data (A, S);
         end loop;
      end loop;

      return Ret;
   end Stepper_Position_To_Position;

   procedure Do_Step (Stepper : Stepper_Name; Data : in out Stepper_Output_Data) is
   begin
      case Data.Current_Step_State is
         when Low_State =>
            Data.Current_Step_State := High_State;
         when High_State =>
            Data.Current_Step_State := Low_State;
      end case;

      Set_Stepper_Pin_State (Stepper, Step_Pin, Data.Current_Step_State);
   end Do_Step;

   procedure Set_Direction (Stepper : Stepper_Name; Dir : Stepgen.Direction; Data : in out Stepper_Output_Data) is
   begin
      case Dir is
         when Stepgen.Forward =>
            Set_Stepper_Pin_State (Stepper, Dir_Pin, High_State);
         when Stepgen.Backward =>
            Set_Stepper_Pin_State (Stepper, Dir_Pin, Low_State);
      end case;
   end Set_Direction;

   procedure Finished_Block (Data : Flush_Extra_Data) is
   begin
      null;
   end Finished_Block;

   Config_Constraint_Error : exception;

   procedure Run is
      Prunt_Params  : My_Config.Prunt_Parameters;
      Error_Message : String := "";
   begin
      My_Config.Config_File.Read (Prunt_Params);

      if Prunt_Params.Enabled then
         begin
            for S in Stepper_Name loop
               declare
                  Stepper_Params : My_Config.Stepper_Parameters;
               begin
                  My_Config.Config_File.Read (Stepper_Params, S);

                  if Stepper_Params.Enabled then
                     Set_Stepper_Pin_State (S, Step_Pin, Low_State);
                     if Stepper_Params.Enabled_On_High then
                        Set_Stepper_Pin_State (S, Enable_Pin, High_State);
                     else
                        Set_Stepper_Pin_State (S, Enable_Pin, Low_State);
                     end if;
                  end if;
               end;
            end loop;

            declare
               Kinematics_Params : My_Config.Kinematics_Parameters;
            begin
               My_Config.Config_File.Read (Kinematics_Params);
               My_Planner.Runner.Setup (In_Scaler => Kinematics_Params.Planning_Scaler);
            end;

            declare
               Data             : Stepper_Pos_Data := [others => [others => Length'Last]];
               Kinematic_Params : My_Config.Kinematics_Parameters;

               Used_Steppers : array (Stepper_Name) of Boolean := [others => False];

               procedure Check_Stepper (S : Stepper_Name) is
                  Stepper_Params : My_Config.Stepper_Parameters;
               begin
                  My_Config.Config_File.Read (Stepper_Params, S);

                  if not Stepper_Params.Enabled then
                     raise Config_Constraint_Error with "Stepper " & S'Image & " attached to an axis but not enabled.";
                  end if;

                  if Used_Steppers (S) then
                     raise Config_Constraint_Error with "Stepper " & S'Image & " attached to multiples axes.";
                  end if;

                  Used_Steppers (S) := True;
               end Check_Stepper;
            begin
               My_Config.Config_File.Read (Kinematic_Params);

               for S in Stepper_Name loop
                  declare
                     Stepper_Params : My_Config.Stepper_Parameters;
                  begin
                     My_Config.Config_File.Read (Stepper_Params, S);

                     case Kinematic_Params.Kind is
                        when My_Config.Cartesian_Kind =>
                           if Kinematic_Params.X_Steppers (S) then
                              Check_Stepper (S);
                              Data (X_Axis, S) :=
                                Stepper_Params.Mm_Per_Step * (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                           end if;

                           if Kinematic_Params.Y_Steppers (S) then
                              Check_Stepper (S);
                              Data (Y_Axis, S) :=
                                Stepper_Params.Mm_Per_Step * (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                           end if;
                        when My_Config.Core_XY_Kind =>
                           if Kinematic_Params.A_Steppers (S) then
                              Check_Stepper (S);
                              Data (X_Axis, S) :=
                                0.5 * Stepper_Params.Mm_Per_Step *
                                (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                              Data (Y_Axis, S) :=
                                0.5 * Stepper_Params.Mm_Per_Step *
                                (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                           end if;

                           if Kinematic_Params.B_Steppers (S) then
                              Check_Stepper (S);
                              Data (X_Axis, S) :=
                                0.5 * Stepper_Params.Mm_Per_Step *
                                (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                              Data (Y_Axis, S) :=
                                -0.5 * Stepper_Params.Mm_Per_Step *
                                (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                           end if;
                     end case;

                     if Kinematic_Params.Z_Steppers (S) then
                        Check_Stepper (S);
                        Data (Z_Axis, S) :=
                          Stepper_Params.Mm_Per_Step * (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                     end if;

                     if Kinematic_Params.E_Steppers (S) then
                        Check_Stepper (S);
                        Data (E_Axis, S) :=
                          Stepper_Params.Mm_Per_Step * (if Stepper_Params.Invert_Direction then -1.0 else 1.0);
                     end if;
                  end;
               end loop;

               My_Stepgen.Preprocessor.Setup (Data);
            end;

            declare
               Params : My_Stepgen.Stepper_Parameters_Array;
            begin
               for S in Stepper_Name loop
                  declare
                     Stepper_Params : My_Config.Stepper_Parameters;
                  begin
                     My_Config.Config_File.Read (Stepper_Params, S);

                     Params (S) :=
                       (Direction_Setup_Time => Time_To_Low_Level (Stepper_Params.Direction_Setup_Time),
                        Step_Time            => Time_To_Low_Level (Stepper_Params.Step_Time),
                        User_Data            => (Current_Step_State => Low_State));
                  end;
               end loop;

               My_Stepgen.Runner.Setup (Params);
            end;

         exception
            when E : Config_Constraint_Error =>
               Error_Message := Ada.Exceptions.Exception_Information (E);
            --  My_Config.Write ((My_Config.Prunt_Parameters'(My_Config.Read) with delta Enabled => False));
         end;
      end if;

      My_GUI.Run (Config_Only => Prunt_Params.Enabled = False or Error_Message /= "", Error_Message => Error_Message);
   end Run;

end Prunt_Glue.Glue;
