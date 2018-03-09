#pragma once

class CameraPose
{
public:
	CameraPose() : mTx(0.0), mTy(0.0), mTz(0.0), mQw(0.0), mQx(0.0), mQy(0.0), mQz(0.0) {}
	float mTx;
	float mTy;
	float mTz;
	float mQw;
	float mQx;
	float mQy;
	float mQz;
};
