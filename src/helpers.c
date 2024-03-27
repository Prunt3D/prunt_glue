void prunt_glue_helper_lock_memory(void)
{
	if (mlockall(MCL_CURRENT | MCL_FUTURE)) {
		fprintf(stderr, "mlockall failed\n");
		exit(1);
	}
}
