
#include "signal_catch.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>


XnBool * g_pValueToSet = NULL;

void SignalHandler(int nSignal)
{
	printf("Caught signal: %d\n", nSignal);
	if (NULL != g_pValueToSet) {
		*g_pValueToSet = true;
	}
}

void CatchSignals(XnBool* bSignalWasCaught)
{
	g_pValueToSet = bSignalWasCaught;

	struct sigaction act;

	memset( &act, 0, sizeof( act ) );
	act.sa_handler = &SignalHandler;
	act.sa_flags = 0;
	sigaction( SIGINT, &act, NULL );
	sigaction( SIGTERM, &act, NULL );
	sigaction( SIGKILL, &act, NULL );
}


