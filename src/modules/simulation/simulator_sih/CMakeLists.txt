# Debt: main builds this as a static library and links the full SIH sources.
# This copy is a host executable so the open-loop rigid-body test can run.

px4_add_module(
	MODULE modules__simulation__simulator_sih
	MAIN simulator_sih
	SRCS
		sih.cpp
	)
