package Prunt_Glue is
   type Stepper_Output_Pins is (Enable_Pin, Step_Pin, Dir_Pin);
   type Stepper_Input_Pins is (Fault_Pin);
   type Pin_State is (High_State, Low_State);
end Prunt_Glue;
