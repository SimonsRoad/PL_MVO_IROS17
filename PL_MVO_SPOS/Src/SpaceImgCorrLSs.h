#pragma once

#include "SpaceLS.h"
#include "ImgLS.h"

class SpaceImgCorrLSs
{
public:
	SpaceImgCorrLSs();
	~SpaceImgCorrLSs();

	SpaceImgCorrLSs(const SpaceLS& spaceLS, const ImgLS& imgLS);

	SpaceLS spaceLS;
	ImgLS imgLS;
};

