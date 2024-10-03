#include <mavlink.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>

union px4_custom_mode {
	struct {
		uint16_t reserved;
		uint8_t main_mode;
		uint8_t sub_mode;
	};
	uint32_t data;
	float data_float;
	struct {
		uint16_t reserved_hl;
		uint16_t custom_mode_hl;
	};
};

// Function prototypes
void send_heartbeat();
void send_gps();
mavlink_message_t msg;
mavlink_status_t status;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;
int sock;
struct sockaddr_in addr;

// Full args
int main(int argc, char *argv[]) {
  
  // Encode a message
  //mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
  // Send autopilot generic, autopilot1, standby
  //mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_STANDBY); 
  mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4, 0, 0, MAV_STATE_STANDBY);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Open socket and send message
  //int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  //struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(14550);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  // Confirm socket opened
  if (sock < 0) {
    printf("Failed to open socket\n");
    return 1;
  }

  // Create heartbeat thread
  pthread_t heartbeat_thread;
  pthread_create(&heartbeat_thread, NULL, send_heartbeat, NULL);

  //Create gps thread
  pthread_t gps_thread;
  pthread_create(&gps_thread, NULL, send_gps, NULL);

  // Send message at 4Hz
  while (1) {
    //printf("Sending message\n");
    //sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    // Receive any messages
    // Enable waitall flag in recvfrom
    int bytes_read;
    //recvfrom(sock, buf, MAVLINK_MAX_PACKET_LEN, 0, NULL, NULL);
    bytes_read = recvfrom(sock, buf, MAVLINK_MAX_PACKET_LEN, 0, NULL, NULL);
    //for (uint16_t i = 0; i < len; i++) {
    //for (uint16_t i = 0; i < MAVLINK_MAX_PACKET_LEN; i++) {
    for (uint16_t i = 0; i < bytes_read; i++) { 
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
      //Check if communication is a command long
      mavlink_command_long_t command_long;
      mavlink_msg_command_long_decode(&msg, &command_long);
      printf("Received command with command ID %d\n", command_long.command); 
      if (command_long.command == MAV_CMD_REQUEST_MESSAGE) {
	// Print received message id
	printf("Received request message of ID %f\n", command_long.param1);
	//Check if PROTOCOL_VERSION is requested
	if (command_long.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
	  printf("Received request autopilot version\n");
	  // Respond with AUTOPILOT_VERSION message
	  const uint8_t * version_bytes = "DanSFirmware";
	  const uint8_t * uid2 = "DanS"; 
	  mavlink_msg_autopilot_version_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, (uint64_t)8223, 2, 2, 2, 52, version_bytes, version_bytes, version_bytes, 45, 1, 1, uid2);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	}else if (command_long.param1 == MAVLINK_MSG_ID_PROTOCOL_VERSION) {
	  printf("Received request protocol version\n");
	  // Respond with PROTOCOL_VERSION message
	  mavlink_msg_protocol_version_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 2, 2, 2, "DanS", "DanS");
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	}/*else if (command_long.param1 == MAVLINK_MSG_ID_AVAILABLE_MODES) {
	  printf("Received request available modes\n");
	  // Respond with AVAILABLE_MODES message
	  mavlink_msg_available_modes_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 2, 1, 1, 0, 1, "First Mode");
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	  mavlink_msg_available_modes_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 2, 2, 2, 0, 1, "Second Mode");
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
      }*/
      }

      printf("Received message with ID %d\n", msg.msgid);
      if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
	printf("Received QGC heartbeat\n");
      // If received parameter request list, send parameter value
      //Check for COMMAND_INT
      }else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
	printf("Received command int\n");
      }else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
	printf("Received parameter request list\n");
	// Send empty parameter value
	mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "test", 0, MAV_PARAM_TYPE_INT32, 0, 0); 
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	// Send SYS_AUTOSTART parameter value of 4011
	//mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "SYS_AUTOSTART", 4001, MAV_PARAM_TYPE_REAL32, 0, 0); 
	mavlink_param_union_t param_value;
	param_value.param_int32 = 4001;
	mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "SYS_AUTOSTART", param_value.param_float, MAV_PARAM_TYPE_INT32, 0, 0); 
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	param_value.param_int32 = 42;
	mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "MAV_SYS_ID", param_value.param_float, MAV_PARAM_TYPE_INT32, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	param_value.param_int32 = 0;
	mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "SYS_AUTOCONFIG", param_value.param_float, MAV_PARAM_TYPE_INT32, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	param_value.param_int32 = 0;
	mavlink_msg_param_value_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, "COM_RC_IN_MODE", param_value.param_float, MAV_PARAM_TYPE_INT32, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
      }

    }/*else{
      // Print reason packet was not parsed 
      printf("Packet not parsed: ID %d\n", msg.msgid);

      // Do frame char buffer to check error
      //mavlink_frame_char(MAVLINK_COMM_0, buf[i], &msg, &status);
      //printf("Parse state: %d\n", status.parse_state);

      


    }*/
  } 
  }

  
  // Decode a message
  for (uint16_t i = 0; i < len; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
      printf("Received message with ID %d\n", msg.msgid);
    }
  }
  
  return 0;
}

void send_heartbeat() {
  while(1) {
	// Send autopilot generic, autopilot1, standby
	//printf("Sending heartbeat\n");
	//mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_STANDBY); 
	//mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4, 0, 0, MAV_STATE_STANDBY);
	//mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4, 13, 32760+counter, MAV_STATE_STANDBY); 
	union px4_custom_mode custom_mode;
	custom_mode.data = 0;
	custom_mode.main_mode = 1;
	custom_mode.sub_mode = 0;
	mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4, 13, custom_mode.data, MAV_STATE_STANDBY); 
	//mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, 12, 0, MAV_STATE_STANDBY); 
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	// Sleep for a quarter second 
	usleep(250000);
  }
}


void send_gps() {
	// Send global origin
	mavlink_msg_gps_global_origin_pack_chan(1, 1, MAVLINK_COMM_0, &msg, 25.649033781863928 * 1E7, -100.28982120113689 * 1E7, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	//int counter = 0;
	while(1) {
	  // Use global position int pack chan
	  //25.649033781863928, -100.28982120113689
	  //mavlink_msg_global_position_int_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 0, (25.649033781863928 * 1E7) + counter, -100.28982120113689 * 1E7, 0, 0, 0, 0, 0, 0);
	  mavlink_msg_global_position_int_pack_chan(1, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, 0, 25.649033781863928 * 1E7, -100.28982120113689 * 1E7, 0, 0, 0, 0, 0, 0);
	  len = mavlink_msg_to_send_buffer(buf, &msg);
	  sendto(sock, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
	  //counter += 100;
	  usleep(1000000);
	}
}
