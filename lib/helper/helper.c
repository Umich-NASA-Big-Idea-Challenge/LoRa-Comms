#include "helper.h"

// Encode node ID (2 bits) and message sequence (6 bits) into message_id
uint32_t encodeMessageId(uint8_t nodeId, uint8_t sequence) {
    // Ensure nodeId uses only 2 bits (0-3)
    nodeId &= 0x03;
    
    // Ensure sequence uses only 6 bits (0-63)
    sequence &= 0x3F;
    
    // Combine: shift nodeId to the high 2 bits, then OR with sequence
    return (nodeId << 6) | sequence;
}

// Decode message_id back into node ID and sequence
void decodeMessageId(uint32_t messageId, uint8_t* nodeId, uint8_t* sequence) {
    // Extract high 2 bits for nodeId
    *nodeId = (messageId >> 6) & 0x03;
    
    // Extract low 6 bits for sequence
    *sequence = messageId & 0x3F;
}

// Encode up to 4 node IDs (each 2 bits) into an 8-bit encrypted path
uint32_t encodeEncryptedPath(uint8_t* nodes, uint8_t nodeCount) {
    uint32_t path = 0;
    
    // Limit to maximum of 4 nodes
    if (nodeCount > 4) nodeCount = 4;
    
    // Each node takes 2 bits
    for (uint8_t i = 0; i < nodeCount; i++) {
        // Ensure each node ID is only 2 bits (0-3)
        uint8_t nodeId = nodes[i] & 0x03;
        
        // Place at the appropriate position (from right to left)
        // Node 0 at bits 0-1, Node 1 at bits 2-3, etc.
        path |= (nodeId << (i * 2));
    }
    
    return path;
}

// Decode the encrypted path back into node IDs
// Returns the number of nodes found
uint8_t decodeEncryptedPath(uint32_t encryptedPath, uint8_t* nodes) {
    uint8_t nodeCount = 0;
    
    // Assume all 4 possible node positions are used
    for (uint8_t i = 0; i < 4; i++) {
        // Extract 2 bits for each node position
        nodes[i] = (encryptedPath >> (i * 2)) & 0x03;
        
        // If we find a non-zero node ID, increase the count
        if (nodes[i] != 0) {
            nodeCount = i + 1;
        }
    }
    
    return nodeCount;
}