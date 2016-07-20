#ifndef CMP_H
#define CMP_H

int compressor_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned int dst, unsigned int deadline);
void * compressor_pop(int port, int timeout, char* & data, unsigned int & size, unsigned char & src, unsigned char & priority);
int compressor_pop_done(void * data);
int compressor_set_enabled(bool enabled);

#endif
