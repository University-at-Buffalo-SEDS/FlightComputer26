#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fcapi.h"
#include "testing.h"

#define id "FS "


static void normal_launch(void)
{
	uint8_t cmd;

	SedsPacketView pkt = {
		.sender = id,
		.sender_len = sizeof id,
		.ty = SEDS_DT_FLIGHT_COMMAND,
		.payload = &cmd,
		.payload_len = sizeof cmd,
	};

	cmd = Compat_Postinit_Signal;
	on_fc_packet(&pkt, NULL);

	tx_thread_sleep(random_wait);

	cmd = Compat_Rollback_Signal;
	on_fc_packet(&pkt, NULL);

	tx_thread_sleep(random_wait);

	cmd = Compat_Postinit_Signal;
	on_fc_packet(&pkt, NULL);

	tx_thread_sleep(random_wait);

	cmd = Compat_Launch_Signal;
	on_fc_packet(&pkt, NULL);
}


static void set_options(void)
{
	uint8_t cmd;

	SedsPacketView pkt = {
		.sender = id,
		.sender_len = sizeof id,
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


static void stripped_groundstation_launch()
{
	uint8_t cmd;

	SedsPacketView pkt = {
		.sender = id,
		.sender_len = sizeof id,
		.ty = SEDS_DT_FLIGHT_COMMAND,
		.payload = &cmd,
		.payload_len = sizeof cmd,
	};

	cmd = Compat_Rollback_Signal;
	on_fc_packet(&pkt, NULL);

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

	for (fu8 k = 0; k < 4; ++k)
	{
		on_fc_packet(&pkt, NULL);
		tx_thread_sleep(random_wait);
	}
}


void emulate_handler_caller(void)
{
	srand(now_ms());
	tx_thread_sleep(random_wait);

	set_options();
	normal_launch();
	stripped_groundstation_launch();
}