#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fcapi.h"
#include "testing.h"

#define id "FS "


static conditional void rollback_after_hitl(void)
{
	store(&sm.flight, Startup, Rel);

	fu32 conf = load(&g_conf, Acq);

	if (!(conf & Using_Ascent_KF))
	{
		ascent_initialize(conf);
	}
}


static conditional void normal_launch(void)
{
	uint8_t cmd;

	SedsPacketView pkt = {
		.sender = id,
		.sender_len = 4,
		.ty = SEDS_DT_FLIGHT_COMMAND,
		.payload = &cmd,
		.payload_len = sizeof cmd,
	};

	cmd = Compat_Postinit_Signal;
	on_fc_packet(&pkt, NULL);

	tx_thread_sleep(5050);

	cmd = Compat_Postinit_Signal;
	on_fc_packet(&pkt, NULL);

	tx_thread_sleep(5050);

	cmd = Compat_Launch_Signal;
	on_fc_packet(&pkt, NULL);
}


static conditional void set_options(void)
{
	uint8_t cmd;

	SedsPacketView pkt = {
		.sender = id,
		.sender_len = 4,
		.ty = SEDS_DT_FLIGHT_COMMAND,
		.payload = &cmd,
		.payload_len = sizeof cmd,
	};

	fu8 k = Compat_Monitor_Altitude;
	for (; k < Compat_Messages; ++k)
	{
		cmd = k;
		on_fc_packet(&pkt, NULL);
		tx_thread_sleep(random_wait);
	}
}


static conditional void stripped_groundstation_launch()
{
	uint8_t cmd;

	SedsPacketView pkt = {
		.sender = id,
		.sender_len = 4,
		.ty = SEDS_DT_FLIGHT_COMMAND,
		.payload = &cmd,
		.payload_len = sizeof cmd,
	};

	tx_thread_sleep(random_wait);

	pkt.ty = SEDS_DT_FLIGHT_STATE;

	for (fu8 k = G_Startup; k < G_Aborted; ++k)
	{
		if (k != G_Armed)
		{
			cmd = k;
			on_fc_packet(&pkt, NULL); /* Nothing should happen */
		}
	}

	cmd = G_Armed;

	for (fu8 k = 0; k < 2; ++k)
	{
		on_fc_packet(&pkt, NULL);
		tx_thread_sleep(5050);

		on_fc_packet(&pkt, NULL);
		tx_thread_sleep(2050);
	}

	pkt.ty = SEDS_DT_FLIGHT_COMMAND;
	cmd = Compat_Launch_Signal;
	on_fc_packet(&pkt, NULL);
}


void emulate_handler_caller(void)
{
	srand(now_ms());
	tx_thread_sleep(random_wait);

	const int scenario = 4;

	if (scenario == 1)
	{
		set_options();
	}
	if (scenario == 2)
	{
		rollback_after_hitl();
		normal_launch();
	}
	if (scenario == 4)
	{
		rollback_after_hitl();
		stripped_groundstation_launch();
	}
}