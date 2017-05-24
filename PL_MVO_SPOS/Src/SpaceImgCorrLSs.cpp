
#include "SpaceImgCorrLSs.h"


SpaceImgCorrLSs::SpaceImgCorrLSs()
{
}


SpaceImgCorrLSs::~SpaceImgCorrLSs()
{
}


SpaceImgCorrLSs::SpaceImgCorrLSs(const SpaceLS& spaceLS, const ImgLS& imgLS)
{
	this->spaceLS = spaceLS;
	this->imgLS = imgLS;
}
