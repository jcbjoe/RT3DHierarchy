//*********************************************************************************************
// File:			Aeroplane.cpp
// Description:		A very simple class to represent an aeroplane as one object with all the
//					hierarchical components stored internally within the class.
// Module:			Real-Time 3D Techniques for Games
// Created:			Jake - 2010-2011
// Notes:
//*********************************************************************************************

#include "Aeroplane.h"

// Initialise static class variables.
CommonMesh* Aeroplane::s_pPlaneMesh = NULL;
CommonMesh* Aeroplane::s_pPropMesh = NULL;
CommonMesh* Aeroplane::s_pTurretMesh = NULL;
CommonMesh* Aeroplane::s_pGunMesh = NULL;

bool Aeroplane::s_bResourcesReady = false;

Aeroplane::Aeroplane(float fX, float fY, float fZ, float fRotY)
{
	m_mWorldMatrix = XMMatrixIdentity();
	m_mPropWorldMatrix = XMMatrixIdentity();
	m_mTurretWorldMatrix = XMMatrixIdentity();
	m_mGunWorldMatrix = XMMatrixIdentity();
	m_mCamWorldMatrix = XMMatrixIdentity();

	m_v4Rot = XMFLOAT4(0.0f, fRotY, 0.0f, 0.0f);
	m_v4Pos = XMFLOAT4(fX, fY, fZ, 0.0f);

	m_v4PropOff = XMFLOAT4(0.0f, 0.0f, 1.9f, 0.0f);
	m_v4PropRot = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);

	m_v4TurretOff = XMFLOAT4(0.0f, 1.05f, -1.3f, 0.0f);
	m_v4TurretRot = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);

	m_v4GunOff = XMFLOAT4(0.0f, 0.5f, 0.0f, 0.0f);
	m_v4GunRot = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);

	m_v4CamOff = XMFLOAT4(0.0f, 4.5f, -15.0f, 0.0f);
	m_v4CamRot = XMFLOAT4(0.0f, 0.0f, 0.0f, 0.0f);

	m_vCamWorldPos = XMVectorZero();
	m_vForwardVector = XMVectorZero();

	m_fSpeed = 0.0f;

	m_bGunCam = false;

}

Aeroplane::~Aeroplane(void)
{
}

void Aeroplane::SetWorldPosition(float fX, float fY, float fZ)
{
	m_v4Pos = XMFLOAT4(fX, fY, fZ, 0.0f);
	UpdateMatrices();
}

void Aeroplane::UpdateMatrices(void)
{
	XMMATRIX mRotX, mRotY, mRotZ, mTrans;
	XMMATRIX mPlaneCameraRot, mForwardMatrix;

	// [START HERE]

	// Calculate m_mWorldMatrix for plane based on Euler rotation angles and position data.
	// i.e. Use XMMatrixRotationX(), XMMatrixRotationY(), XMMatrixRotationZ() and XMMatrixTranslationFromVector to calculate mRotX, mRotY, mRotZ and mTrans from m_v4Rot
	// Then concatenate the matrices to calculate m_mWorldMatrix
	mRotX = XMMatrixRotationX(XMConvertToRadians(m_v4Rot.x));
	mRotY = XMMatrixRotationY(XMConvertToRadians(m_v4Rot.y));
	mRotZ = XMMatrixRotationZ(XMConvertToRadians(m_v4Rot.z));
	mTrans = XMMatrixTranslationFromVector(XMLoadFloat4(&m_v4Pos));
	
	m_mWorldMatrix =   mRotZ * mRotX * mRotY * mTrans;

	// [Skip this step first time through] Also calculate mPlaneCameraRot which ignores rotations in Z and X for the camera to parent to
	mPlaneCameraRot = XMMatrixMultiply(mRotY, mTrans);

	// [Skip this step first time through] Get the forward vector out of the world matrix and put it in m_vForwardVector
	m_vForwardVector = m_mWorldMatrix.r[2];

	// Calculate m_mPropWorldMatrix for propeller based on Euler rotation angles and position data.
	// Parent the propeller to the plane

	m_mPropWorldMatrix = XMMatrixRotationZ(XMConvertToRadians(m_v4PropRot.z))*
						 XMMatrixRotationX(XMConvertToRadians(m_v4PropRot.x)) *
						 XMMatrixRotationY(XMConvertToRadians(m_v4PropRot.y)) *
						 XMMatrixTranslationFromVector(XMLoadFloat4(&m_v4PropOff)) *
						 m_mWorldMatrix;

	// Calculate m_mTurretWorldMatrix for propeller based on Euler rotation angles and position data.
	// Parent the turret to the plane
	m_mTurretWorldMatrix = XMMatrixRotationZ(XMConvertToRadians(m_v4TurretRot.z)) *
						   XMMatrixRotationX(XMConvertToRadians(m_v4TurretRot.x)) *
						   XMMatrixRotationY(XMConvertToRadians(m_v4TurretRot.y)) *
						   XMMatrixTranslationFromVector(XMLoadFloat4(&m_v4TurretOff)) *
						   m_mWorldMatrix;

	// Calculate m_mGunWorldMatrix for gun based on Euler rotation angles and position data.
	// Parent the gun to the turret
	m_mGunWorldMatrix = XMMatrixRotationZ(XMConvertToRadians(m_v4GunRot.z)) *
						XMMatrixRotationX(XMConvertToRadians(m_v4GunRot.x)) *
						XMMatrixRotationY(XMConvertToRadians(m_v4GunRot.y)) *
						XMMatrixTranslationFromVector(XMLoadFloat4(&m_v4GunOff)) *
						m_mTurretWorldMatrix;

	// Calculate m_mCameraWorldMatrix for camera based on Euler rotation angles and position data.
	m_mCamWorldMatrix = XMMatrixRotationZ(XMConvertToRadians(m_v4CamRot.z)) *
						XMMatrixRotationX(XMConvertToRadians(m_v4CamRot.x)) *
						XMMatrixRotationY(XMConvertToRadians(m_v4CamRot.y)) *
						XMMatrixTranslationFromVector(XMLoadFloat4(&m_v4CamOff));

	// [Skip this step first time through] Switch between parenting the camera to the plane (without X and Z rotations) and the gun based on m_bGunCam


	if (m_bGunCam) { 
		m_mCamWorldMatrix = XMMatrixMultiply(m_mCamWorldMatrix, m_mGunWorldMatrix);
		
	}
	else {
		m_mCamWorldMatrix = XMMatrixMultiply(m_mCamWorldMatrix, mPlaneCameraRot);
	}

	// Get the camera's world position (m_vCamWorldPos) out of m_mCameraWorldMatrix
	XMVECTOR Scale;
	XMVECTOR Rot;
	XMVECTOR Trans;
	XMMatrixDecompose(&Scale, &Rot, &Trans, m_mCamWorldMatrix);
	m_vCamWorldPos = Trans;

}

XMFLOAT3 planeRotations = XMFLOAT3(0, 0, 0);

void Aeroplane::Update(bool bPlayerControl)
{
	// DON'T DO THIS UNTIL YOu HAVE COMPLETED THE FUNCTION ABOVE
	if(bPlayerControl){

		// Step 1: Make the plane pitch upwards when you press "Q" and return to level when released
		// Maximum pitch = 60 degrees
		if (Application::s_pApp->IsKeyPressed('Q')) {
			if (planeRotations.x + 2 < 60) {
				m_v4Rot.x += 2;
				planeRotations.x += 2;
			}
		}
		
		// Step 2: Make the plane pitch downwards when you press "A" and return to level when released
		// You can also impose a take off speed of 0.5 if you like
		// Minimum pitch = -60 degrees
		if (Application::s_pApp->IsKeyPressed('A')) {
			if (planeRotations.x - 2 > -60) {
				m_v4Rot.x -= 2;
				planeRotations.x -= 2;
			}
			
		}
		if (!Application::s_pApp->IsKeyPressed('Q') && !Application::s_pApp->IsKeyPressed('A')) {
			if (planeRotations.x != 0) {
				if (planeRotations.x > 0) {
					planeRotations.x -= 2;
					m_v4Rot.x -= 2;
				}
				else {
					planeRotations.x += 2;
					m_v4Rot.x += 2;
				}
			}
		}

		// Step 3: Make the plane yaw and roll left when you press "O" and return to level when released
		// Maximum roll = 20 degrees
		if (Application::s_pApp->IsKeyPressed('O')) {
			
			if (planeRotations.z + 1 < 20) {
				m_v4Rot.z += 1;
				planeRotations.z += 1;
			}
			m_v4Rot.y -= 1;
		}

		// Step 4: Make the plane yaw and roll right when you press "P" and return to level when released
		// Minimum roll = -20 degrees
		if (Application::s_pApp->IsKeyPressed('P')) {
			if (planeRotations.z - 1 > -20) {
				m_v4Rot.z -= 1;
				planeRotations.z -= 1;
			}
			m_v4Rot.y += 1;
		}

		if (!Application::s_pApp->IsKeyPressed('O') && !Application::s_pApp->IsKeyPressed('P')) {
			if (planeRotations.z != 0) {
				if (planeRotations.z > 0) {
					planeRotations.z -= 1;
					m_v4Rot.z -= 1;
				}
				else {
					planeRotations.z += 1;
					m_v4Rot.z += 1;
				}
			}
		}

	} // End of if player control

	// Apply a forward thrust and limit to a maximum speed of 1
	m_fSpeed += 0.001f;

	if(m_fSpeed > 1)
		m_fSpeed = 1;

	// Rotate propeller and turret
	m_v4PropRot.z += 100 * m_fSpeed;
	m_v4TurretRot.y += 0.1f;

	// Tilt gun up and down as turret rotates
	m_v4GunRot.x = (sin((float)XMConvertToRadians(m_v4TurretRot.y * 4.0f)) * 10.0f) - 10.0f;

	UpdateMatrices();

	// Move Forward
	XMVECTOR vCurrPos = XMLoadFloat4(&m_v4Pos);
	vCurrPos += m_vForwardVector * m_fSpeed;
	XMStoreFloat4(&m_v4Pos, vCurrPos);
}

void Aeroplane::LoadResources(void)
{
	s_pPlaneMesh = CommonMesh::LoadFromXFile(Application::s_pApp, "Resources/Plane/plane.x");
	s_pPropMesh = CommonMesh::LoadFromXFile(Application::s_pApp, "Resources/Plane/prop.x");
	s_pTurretMesh = CommonMesh::LoadFromXFile(Application::s_pApp, "Resources/Plane/turret.x");
	s_pGunMesh = CommonMesh::LoadFromXFile(Application::s_pApp, "Resources/Plane/gun.x");
}

void Aeroplane::ReleaseResources(void)
{
	delete s_pPlaneMesh;
	delete s_pPropMesh;
	delete s_pTurretMesh;
	delete s_pGunMesh;
}

void Aeroplane::Draw(void)
{
	Application::s_pApp->SetWorldMatrix(m_mWorldMatrix);
	s_pPlaneMesh->Draw();

	Application::s_pApp->SetWorldMatrix(m_mPropWorldMatrix);
	s_pPropMesh->Draw();

	Application::s_pApp->SetWorldMatrix(m_mTurretWorldMatrix);
	s_pTurretMesh->Draw();

	Application::s_pApp->SetWorldMatrix(m_mGunWorldMatrix);
	s_pGunMesh->Draw();
}
