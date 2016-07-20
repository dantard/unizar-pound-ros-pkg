
#include <bzlib.h>
#include <string.h>
#include <iostream>
#include "interface/interface.h"
#include "ShapeShifterMessage.h"

struct cmp_header_t{
    unsigned int uncmp_size;
    char data[0];
};

static int m_enabled = 0;


int compressor_set_enabled(bool enabled){
    m_enabled = enabled;
}
int compressor_push(char * data, unsigned int size, unsigned char priority, unsigned char port, unsigned char src, unsigned int dst, unsigned int deadline){

    WrapperHeader * wh = (WrapperHeader *) data;
    wh->uncompressed_size = 0;

    if (m_enabled){
        unsigned int payload = size - sizeof(WrapperHeader);
        unsigned int cmp_size = payload;

        char * buff = new char [cmp_size];
        int bz_result = BZ2_bzBuffToBuffCompress(buff, &cmp_size, &wh->data[0], payload, 1, 0, 30);

        if (bz_result == BZ_OUTBUFF_FULL){
            ROS_INFO_ONCE("Port %d -> negative compression, sending uncompressed", port);
        }else if (bz_result != 0){
            ROS_WARN_THROTTLE(1000,"Port %d -> compression error (%d), sending uncompressed", port, bz_result);
        }else{
            wh->uncompressed_size = payload;
            memcpy(&wh->data[0], buff, cmp_size);
            size = cmp_size + sizeof(WrapperHeader);
        }
        delete [] buff;
    }
    return wrapper_push(data, size, priority, port, src,dst,deadline);
}

void * compressor_pop(int port, int timeout, char* & data, unsigned int & size, unsigned char & src, unsigned char & priority){


    std::pair<void *, char * > * pnt = new std::pair<void *, char *>();
    pnt->first = wrapper_pop(port, timeout, data, size, src, priority);
    pnt->second = 0;

    WrapperHeader * wh1  = (WrapperHeader *) data;

    if (wh1->uncompressed_size >0){
        WrapperHeader * wh2 = (WrapperHeader *) new char[wh1->uncompressed_size + sizeof(WrapperHeader)];
        int bz_result = BZ2_bzBuffToBuffDecompress(&wh2->data[0], &wh1->uncompressed_size, &wh1->data[0], size - sizeof(WrapperHeader), 0, 0);
        if (bz_result != 0){
            ROS_ERROR_THROTTLE(1000,"Port %d -> decompression error (%d), sending uncompressed", port, bz_result);
            pnt->first = 0;
        }else{
            data = pnt->second = (char *) wh2;
            size = wh1->uncompressed_size;
            memcpy(wh2, wh1, sizeof(WrapperHeader));
        }
    }
    return pnt;
}

int compressor_pop_done(void * data){
    std::pair<void *, char * > * pnt = (std::pair<void *, char *> *) data;
    if (pnt->second){
        delete pnt->second;
    }
    return wrapper_pop_done(pnt->first);
}

