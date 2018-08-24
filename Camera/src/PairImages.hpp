#ifndef PAIRIMAGES_HPP_
#define PAIRIMAGES_HPP_

#include "Images.hpp"

namespace ScanVan {

class PairImages {
	Images img0;
	Images img1;

public:
	PairImages();
	PairImages(Images &&a, Images &&b);
	void showPair();
	virtual ~PairImages();
};

} /* namespace ScanVan */

#endif /* PAIRIMAGES_HPP_ */
