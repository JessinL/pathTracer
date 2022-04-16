#ifndef WIRTE_JPG_H_
#define WIRTE_JPG_H_

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
int writejpg(char const *filename, int x, int y, int comp, const void *data, int quality){
    return stbi_write_jpg(filename, x, y, comp, data, quality);
}

#endif