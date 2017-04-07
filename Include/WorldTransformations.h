#pragma once

#include <Eigen/Dense>
#include <cmath>

const float PI = 3.14159265f; 

struct PersProjInfo
{
	PersProjInfo(
					float FOV_ = 0.0f, 
					float Width_ = 0.0f, 
					float Height_ = 0.0f, 
					float zNear_ = 0.0f, 
					float zFar_ = 0.0f
				)
		: FOV(FOV_), Width(Width_), Height(Height_), zNear(zNear_), zFar(zFar_) {};

    float FOV;
    float Width; 
    float Height;
    float zNear;
    float zFar;
};

class WorldTransformation
{
public:
	WorldTransformation();

	void ScaleTrans(float x, float y, float z);

	void RotationTrans(float xdegs, float ydegs, float zdegs);

	void TranslationTrans(float x, float y, float z);
	
	void SetPersProjInfo(const PersProjInfo &p);

	void PersProjTrans(bool isOn);
	
	Eigen::Matrix4f GetTransformation();

	void ResetTransformation();

	const Eigen::Matrix4f& GetScale();
	const Eigen::Matrix4f& GetRotation();
	const Eigen::Matrix4f& GetTranslation();
	const Eigen::Matrix4f& GetPerspective();

private:
	PersProjInfo persProj;

	Eigen::Matrix4f m_Scale;
	Eigen::Matrix4f m_Rotation;
	Eigen::Matrix4f m_Translation;
	Eigen::Matrix4f m_Perspective;
};