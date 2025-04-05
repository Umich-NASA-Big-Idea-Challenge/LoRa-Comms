#ifndef HELPER_H
#define HELPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Function to encode node ID and sequence into message_id
uint32_t encodeMessageId(uint8_t nodeId, uint8_t sequence);

// Function to decode message_id into node ID and sequence
void decodeMessageId(uint32_t messageId, uint8_t* nodeId, uint8_t* sequence);

// Function to encode the path (up to 4 nodes, 2 bits each)
uint32_t encodeEncryptedPath(uint8_t* nodes, uint8_t nodeCount);

// Function to decode the encrypted path
uint8_t decodeEncryptedPath(uint32_t encryptedPath, uint8_t* nodes);

#ifdef __cplusplus
}
#endif


#endif /* HELPER_H */