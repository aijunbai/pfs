/***************************************************************************
 *   cameraModel.h     - description
 *
 *   This program is part of the ETISEO project.
 *
 *   See http://www.etiseo.net  http://www.silogic.fr    
 *
 *   (C) Silogic - ETISEO Consortium
 ***************************************************************************/



#ifndef _CAMERA_MODEL_OBJECT_H_
#define _CAMERA_MODEL_OBJECT_H_

#include "xmlUtil.h"
#include "geometry.h"
#include <QtCore/qpoint.h>

#define XML_TAG_CAMERA			BAD_CAST"Camera"
#define XML_TAG_NAME			BAD_CAST"name"


#define XML_TAG_GEOMETRY		BAD_CAST"Geometry"
#define XML_TAG_WIDTH			BAD_CAST"width"
#define XML_TAG_HEIGHT			BAD_CAST"height"
#define XML_TAG_NCX				BAD_CAST"ncx"
#define XML_TAG_NFX				BAD_CAST"nfx"
#define XML_TAG_DX				BAD_CAST"dx"
#define XML_TAG_DY				BAD_CAST"dy"
#define XML_TAG_DPX				BAD_CAST"dpx"
#define XML_TAG_DPY				BAD_CAST"dpy"

#define XML_TAG_INTRINSIC		BAD_CAST"Intrinsic"
#define XML_TAG_FOCAL			BAD_CAST"focal" 
#define XML_TAG_KAPPA1			BAD_CAST"kappa1" 
#define XML_TAG_CX				BAD_CAST"cx" 
#define XML_TAG_CY				BAD_CAST"cy" 
#define XML_TAG_SX				BAD_CAST"sx" 

#define XML_TAG_EXTRINSIC		BAD_CAST"Extrinsic"
#define XML_TAG_TX				BAD_CAST"tx" 
#define XML_TAG_TY				BAD_CAST"ty" 
#define XML_TAG_TZ				BAD_CAST"tz" 
#define XML_TAG_RX				BAD_CAST"rx" 
#define XML_TAG_RY				BAD_CAST"ry" 
#define XML_TAG_RZ				BAD_CAST"rz" 


namespace Etiseo {
	
 //!  A root class handling camera model
 class CameraModel
 {
    public:
      
	  //! Constructor
      CameraModel();
      //! Destructor
      virtual ~CameraModel();
	  
	  //! Access to members
	  std::string name() { return mName; }
	  const std::string& name() const { return mName; }
	  void setName(const std::string& name) { mName = name; }
	  
	  inline int width() const { return mImgWidth; }
	  inline int height() const { return mImgHeight; }
	  inline double ncx() const { return mNcx; }
	  inline double nfx() const { return mNfx; }
	  inline double dx() const { return mDx; }
	  inline double dy() const { return mDy; }
	  inline double dpx() const { return mDpx; }
	  inline double dpy() const { return mDpy; }
	  inline double cx() const { return mCx; }
	  inline double cy() const { return mCy; }
	  inline double sx() const { return mSx; }
	  inline double focal() const { return mFocal; }
	  inline double kappa1() const { return mKappa1; }
	  inline double tx() const { return mTx; }
	  inline double ty() const { return mTy; }
	  inline double tz() const { return mTz; }
	  inline double rx() const { return mRx; }
	  inline double ry() const { return mRy; }
	  inline double rz() const { return mRz; }
	  inline double cposx() const { return mCposx; }
	  inline double cposy() const { return mCposy; }
	  inline double cposz() const { return mCposz; }
	  
	  void setGeometry(int width, int height, double ncx, double nfx, 
	  					double dx, double dy, double dpx, double dpy);
		
	  void setIntrinsic(double focal, double kappa1, double cx, double cy, double sx);
	  
	  void setExtrinsic(double tx, double ty, double tz, double rx, double ry, double rz);
	   
	  //! Loading from an XML
	  virtual bool fromXml(std::istream& is);
	  //! Saving to an XML
	  virtual void toXml(std::ostream& os) const;
	  
	  //! Loading from a .dat files = output of the Tsai calibration 
	  virtual bool fromDat(std::istream& is, int width, int height);
	  
	  //! Coordinate manipulation
	  //! from image coordinate to world coordinate
	  bool imageToWorld(double Xi, double Yi, double Zw, double& Xw, double &Yw);
	  
	  //! from world coordinate to image coordinate
	  bool worldToImage(double Xw, double Yw, double Zw, double& Xi, double& Yi);
	  
	  //! convert from undistorted to distorted image
	  bool undistortedToDistortedImageCoord (double Xfu, double Yfu, double& Xfd, double& Yfd);
	  //! convert from distorted to undistorted image
	  bool distortedToUndistortedImageCoord (double Xfd, double Yfd, double& Xfu, double& Yfu);
	  
	  //! from world coordinate to camera coordinate
	  bool worldToCameraCoord (double xw, double yw, double zw, double& xc, double& yc, double& zc);
	  //! from camera coordinate to world coordinate
	  bool cameraToWorldCoord (double xc, double yc, double zc, double& xw, double& yw, double& zw);

	  QPointF GroundToImage(double gx, double gy) {
	    double ix = 0.0, iy = 0.0;
	    worldToImage(
	        gx * 1000.0, //m -> mm
	        gy * 1000.0, //m -> mm
	        0.0, ix, iy);

	    return QPointF(ix, iy);
	  }

	  vector2d ImageToGround(double ix, double iy) {
	    double gx = 0.0, gy = 0.0, gz = 0.0;
	    imageToWorld(ix, iy, gz, gx, gy);

	    return vector2d(gx, gy) / 1000.0; //mm -> m
	  }

	protected:
	
		virtual void internalInit();
		
		//! Coordinate manipulation, intermediate transformation :
		//! convert from distorted to undistorted sensor plane coordinates 
		void distortedToUndistortedSensorCoord (double Xd, double Yd, double& Xu, double& Yu);
		//! convert from undistorted to distorted sensor plane coordinates
		void undistortedToDistortedSensorCoord (double Xu, double Yu, double& Xd, double& Yd);
		
	private:
		
		bool			isInit;
		std::string		mName;
		
		// geometry
		int				mImgWidth;
		int				mImgHeight;
		double			mNcx;
		double			mNfx;
		double			mDx;
		double			mDy;
		double			mDpx;
		double			mDpy;

		// intrinsic 
		double			mFocal;
		double			mKappa1;
		double			mCx;
		double			mCy;
		double			mSx;
 
		// extrinsic 
		double			mTx;
		double			mTy;
		double			mTz;
		double			mRx;
		double			mRy;
		double			mRz;
		
		// for computation
		double			mR11;
		double			mR12;
		double			mR13;
		double			mR21;
		double			mR22;
		double			mR23;
		double			mR31;
		double			mR32;
		double			mR33;
		
		//camera position
		double			mCposx;
		double			mCposy;
		double			mCposz;
 };
};

#endif
