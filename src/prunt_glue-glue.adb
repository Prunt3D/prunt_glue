package body Prunt_Glue.Glue is

   procedure Helper_Lock_Memory with
     Import => True, Convention => C, External_Name => "prunt_glue_helper_lock_memory";

   procedure Run is
   begin
      My_GUI.Run;
   end Run;

end Prunt_Glue.Glue;
