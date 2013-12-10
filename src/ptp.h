/*
 * ptp.h
 *
 *  Created on: May 26, 2013
 *      Author: kgamiel
 */

#ifndef PTP_H_
#define PTP_H_

#define PTP_UDP_PACKET_MAX 1472
#define PTP_HEARTBEAT_TTL_S 1
#define PTP_DEFAULT_CLIENT_PORT 50000
#define PTP_DEFAULT_SERVER_PORT 50001
#define PTP_DEFAULT_SERVER_HOST "127.0.0.1"
#define PTP_DEFAULT_CLIENT_HOST "127.0.0.1"

typedef struct __attribute__ ((packed)) {
    unsigned int id;
    double position[4];
    short particle_type;
} ptp_particle_data_t;

#define PTP_PACKET_HEADER_SIZE ((2 * sizeof(unsigned int)) + (7 * sizeof(float)))
#define PTP_PARTICLES_PER_PACKET ((PTP_UDP_PACKET_MAX - PTP_PACKET_HEADER_SIZE) / sizeof(ptp_particle_data_t))

typedef struct __attribute__ ((packed)) {
    unsigned int total_particle_count;
    unsigned int particle_count;
    float t;
    float world_origin[3];
    float world_size[3];
    ptp_particle_data_t data[PTP_PARTICLES_PER_PACKET];
} ptp_packet_t;

typedef struct __attribute__ ((packed)) {
	unsigned int count;
} ptp_heartbeat_packet_t;


#endif /* PTP_H_ */
