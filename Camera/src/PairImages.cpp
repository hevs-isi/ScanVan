#include "PairImages.hpp"

namespace ScanVan {

PairImages::PairImages() {
	// TODO Auto-generated constructor stub

}

PairImages::PairImages(Images &&a, Images &&b) : img0 {std::move(a)}, img1 {std::move(b)} {
}

void PairImages::showPair() {
	img0.show("0");
	img1.show("1");
}

PairImages::~PairImages() {
	// TODO Auto-generated destructor stub
}

} /* namespace ScanVan */
