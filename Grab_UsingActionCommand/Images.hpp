#ifndef IMAGES_HPP_
#define IMAGES_HPP_

#include <vector>
#include <stdint.h>

class Images {
private:
	std::vector<uint8_t> * p_img;
	size_t height = 3008;
	size_t width = 3008;
public:
	Images();
	Images(char * p);
	Images(size_t h, size_t w);
	Images(size_t h, size_t w, char * p);
	virtual ~Images();
};

Images::Images(){
	p_img = new std::vector<uint8_t> {};
}

Images::Images(char * p){
	p_img = new std::vector<uint8_t> {};
	p_img->assign(p,p+(height*width));
}

Images::Images(size_t h, size_t w) : height{h}, width{w} {
	p_img = new std::vector<uint8_t> {};
}

Images::Images(size_t h, size_t w, char * p) : height{h}, width{w} {
	p_img = new std::vector<uint8_t> {};
	p_img->assign(p, p+(height*width));
}

Images::~Images() {
	delete p_img;
}

#endif /* IMAGES_HPP_ */
