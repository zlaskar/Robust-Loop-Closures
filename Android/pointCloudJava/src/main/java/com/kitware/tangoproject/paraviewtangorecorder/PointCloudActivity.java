/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Additional changes are made by Kitware Inc. and are also licensed under
 * the Apache License, Version 2.0 (the "License"):
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 */

package com.kitware.tangoproject.paraviewtangorecorder;

import android.app.Activity;

import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ColorMatrix;
import android.graphics.ColorMatrixColorFilter;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Point;
import android.graphics.SurfaceTexture;

import android.net.Uri;

import android.opengl.Matrix;

import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;

import android.support.v4.content.FileProvider;

import android.util.Log;

import android.view.View;
import android.view.View.OnClickListener;

import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

//import com.google.atap.tango.ux.SplashLayout;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoTextureCameraPreview;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.tangoutils.ModelMatCalculator;

import com.google.atap.tango.ux.TangoUx;
import com.google.atap.tango.ux.TangoUxLayout;
import com.google.atap.tango.ux.TangoUx.StartParams;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.channels.FileChannel;

import java.text.DateFormat;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;
import android.os.Handler;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the
 * {@link Tango} service and propagation of Tango XyzIj data to OpenGL and
 * Layout views. OpenGL rendering logic is delegated to the {@link PCrenderer}
 * class.
 */
public class PointCloudActivity extends Activity implements OnClickListener {

    private static final String TAG = PointCloudActivity.class.getSimpleName();
    private static final double SECS_TO_MILLISECS = 1000.0;
    private Tango mTango;
    private TangoConfig mConfig;

    //private PCRenderer mRenderer;
    //private RGBRenderer mRenderer;
    private final EmptyRenderer mRenderer =  new EmptyRenderer();
    //private GLSurfaceView mGLView;
    private TangoTextureCameraPreview mCameraTexture;
    private double mCameraTime = 0;
    //private static int mCameraID = TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE;
    private static int mCameraID = TangoCameraIntrinsics.TANGO_CAMERA_COLOR;
    private static Bitmap mBitmap;

    private TextView mDeltaTextView;
    private TextView mPoseCountTextView;
    private TextView mPoseTextView;
    private TextView mQuatTextView;
    private TextView mPoseStatusTextView;
    private TextView mTangoEventTextView;
    private TextView mPointCountTextView;
    private TextView mTangoServiceVersionTextView;
    private TextView mApplicationVersionTextView;
    private TextView mAverageZTextView;
    private TextView mFrequencyTextView;

    private Button mFirstPersonButton;
    private Button mThirdPersonButton;
    private Button mTopDownButton;

    private int count;
    private int mPreviousPoseStatus;
    private int mPointCount;
    private double mDeltaTime;
    private double mPosePreviousTimeStamp;
    private double mXyIjPreviousTimeStamp;
    private double mCurrentTimeStamp;
    private double mPointCloudFrameDelta;
    private String mServiceVersion;
    private boolean mIsTangoServiceConnected;
    private boolean mIsTangoCameraConnected;
    private TangoPoseData mPose;

    private TangoUx mTangoUx;
    private TangoUxLayout mTangoUxLayout;

    private static final int UPDATE_INTERVAL_MS = 100;
    public final static Object poseLock = new Object();
    public final static Object depthLock = new Object();
    public final static Object colorLock = new Object();
    public final static Object fileLock = new Object();
    public final static Object cameraLock = new Object();

    // My variables
    private int mValidPoseCallbackCount;
    private Button mTakeSnapButton;
    private TextView mFilesWrittenToSDCardTextView;
    private Switch mAutoModeSwitch;
    private Switch mRecordSwitch;
    private ProgressBar mWaitingProgressBar;
    private TextView mWaitingTextView;
    private LinearLayout mWaitingLinearLayout;

    private static final String mMainDirPath = Environment.getExternalStorageDirectory()
            .getAbsolutePath() + "/Tango/";
    private static final String mSaveDirAbsPath = Environment.getExternalStorageDirectory()
            .getAbsolutePath() + "/Tango/MyPointCloudData/";
    private String mFilename;
    private int mNumberOfFilesWritten;
    private int mNumberOfFrames;
    private Boolean mTimeToTakeSnap;
    private Boolean mAutoMode;
    private String mNowTimeString;
    private ArrayList<float[]> mPosePositionBuffer;
    private ArrayList<float[]> mPoseOrientationBuffer;
    private ArrayList<Float> mPoseTimestampBuffer;
    private ArrayList<String> mFilenameBuffer;
    private float[] cam2dev_Transform;
    private int mNumPoseInSequence;
    AtomicBoolean mIsRecording = new AtomicBoolean(false);
    private int mXyzIjCallbackCount;
    private Semaphore mutex_on_mIsRecording;
    private boolean mAppIsStarting;

    private AtomicBoolean mBitmapReady = new AtomicBoolean(true);
    private AtomicBoolean mPointCloudReady = new AtomicBoolean(true);
    private AtomicBoolean mIsCompressing = new AtomicBoolean(false);

    private static final File mainDir = new File(mMainDirPath);
    private static final File dir = new File(mSaveDirAbsPath);
    // End of My variables

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //getWindow().setFormat(PixelFormat.RGBA_8888);
        setContentView(R.layout.activity_jpoint_cloud);
        setTitle(R.string.app_name);

        mIsTangoCameraConnected = false;
        //mCameraTexture = new TangoTextureCameraPreview(this);
        mCameraTexture = (TangoTextureCameraPreview) findViewById(R.id.textureView);

        //LinearLayout layout = (LinearLayout) findViewById(R.id.cameralayout);
        //layout.addView(mCameraTexture);

        //myCameraTexture.setLayoutParams(new FrameLayout.LayoutParams(1280, 720, Gravity.CENTER));
        //GLES20.glDisable(GLES20.GL_DITHER);

        mConfig = new TangoConfig();
        mTango = new Tango(this);

        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_HIGH_RATE_POSE, false);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_SMOOTH_POSE, true);

        mTangoUxLayout = (TangoUxLayout) findViewById(R.id.layout_tango);
        mTangoUx = new TangoUx(this);
        mTangoUx.setLayout(mTangoUxLayout);
        //mTangoUx.setUxExceptionEventListener(mUxExceptionListener);

        mPoseTextView = (TextView) findViewById(R.id.pose);
        mQuatTextView = (TextView) findViewById(R.id.quat);
        mPoseCountTextView = (TextView) findViewById(R.id.posecount);
        mDeltaTextView = (TextView) findViewById(R.id.deltatime);
        mTangoEventTextView = (TextView) findViewById(R.id.tangoevent);
        mPoseStatusTextView = (TextView) findViewById(R.id.status);
        mPointCountTextView = (TextView) findViewById(R.id.pointCount);
        mTangoServiceVersionTextView = (TextView) findViewById(R.id.version);
        mApplicationVersionTextView = (TextView) findViewById(R.id.appversion);
        mAverageZTextView = (TextView) findViewById(R.id.averageZ);
        mFrequencyTextView = (TextView) findViewById(R.id.frameDelta);

        // Display the version of Tango Service
        mServiceVersion = mConfig.getString("tango_service_library_version");
        mTangoServiceVersionTextView.setText(mServiceVersion);

        //mFirstPersonButton = (Button) findViewById(R.id.first_person_button);
        //mFirstPersonButton.setOnClickListener(this);
        //mThirdPersonButton = (Button) findViewById(R.id.third_person_button);
        //mThirdPersonButton.setOnClickListener(this);
        mTopDownButton = (Button) findViewById(R.id.top_down_button);
        mTopDownButton.setOnClickListener(this);

        //int maxDepthPoints = mConfig.getInt("max_point_cloud_elements");
        //mRenderer = new PCRenderer(maxDepthPoints);
        //mRenderer = new CameraRenderer(maxDepthPoints,this);
        /*mRenderer = new RGBRenderer(this);
        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
        mGLView.setEGLContextClientVersion(2);
        mGLView.setPreserveEGLContextOnPause(true);
        mGLView.setRenderer(mRenderer);
        mGLView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);*/

       //ViewGroup.LayoutParams layoutParams=mGLView.getLayoutParams();
        //layoutParams.width = 1280;
        //layoutParams.height = 720;
        //mGLView.setLayoutParams(layoutParams);

        PackageInfo packageInfo;
        try {
            packageInfo = this.getPackageManager().getPackageInfo(
                    this.getPackageName(), 0);
            mApplicationVersionTextView.setText(packageInfo.versionName);
        } catch (NameNotFoundException e) {
            e.printStackTrace();
        }

        mIsTangoServiceConnected = false;
        startUIThread();

        // My initializations
        mTakeSnapButton = (Button) findViewById(R.id.take_snap_button);
        mTakeSnapButton.setOnClickListener(this);
        mFilesWrittenToSDCardTextView = (TextView) findViewById(R.id.fileWritten);
        mAutoModeSwitch = (Switch) findViewById(R.id.auto_mode_switch);
        mAutoModeSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                autoMode_SwitchChanged(isChecked);
            }
        });
        mRecordSwitch = (Switch) findViewById(R.id.record_switch);
        mRecordSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                record_SwitchChanged(isChecked);
            }
        });
        mWaitingProgressBar = (ProgressBar) findViewById(R.id.progressBar);
        mWaitingProgressBar.setVisibility(View.VISIBLE);
        mWaitingTextView = (TextView) findViewById(R.id.waitingTextView);
        mWaitingTextView.setVisibility(View.VISIBLE);
        mWaitingTextView.setText(R.string.waitInitialize);
        mWaitingLinearLayout = (LinearLayout) findViewById(R.id.waitingBarLayout);
        mWaitingLinearLayout.setVisibility(View.GONE);

        mFilename = "";
        mNumberOfFilesWritten = 0;
        mTimeToTakeSnap = false;
        mTakeSnapButton.setEnabled(false);
        mAutoMode = false;
        mAutoModeSwitch.setChecked(false);
        mIsRecording.set(false);
        mRecordSwitch.setChecked(false);
        mPosePositionBuffer = new ArrayList<float[]>();
        mPoseOrientationBuffer = new ArrayList<float[]>();
        mPoseTimestampBuffer = new ArrayList<Float>();
        mFilenameBuffer = new ArrayList<String>();
        mNumPoseInSequence = 0;
        mXyzIjCallbackCount = 0;
        mutex_on_mIsRecording = new Semaphore(1,true);
        mAppIsStarting = true;

        if(!mainDir.exists()) {
            boolean created = mainDir.mkdir();
            if (created) {
                Log.i(TAG, "Folder: \"" + mMainDirPath + "\" created\n");
            }
        }

        if(!dir.exists()) {
            boolean created = dir.mkdir();
            if (created) {
                Log.i(TAG, "Folder: \"" + mSaveDirAbsPath + "\" created\n");
            }
        }
        // End of My initializations
    }

    /*@Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        //myCameraTexture.setLayoutParams(new FrameLayout.LayoutParams(
        //        720, 1280, Gravity.CENTER));

    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {

        return true;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // Invoked every time there's a new Camera preview frame

    }*/

    private void setupCamera() {
        if (mCameraID == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
            mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, false);
        }
        else
        {
            mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, false);
        }

        //mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_COLORMODEAUTO, false);
        //mConfig.putInt(TangoConfig.KEY_INT_COLOREXP, 10000000); //ms
        //mConfig.putInt(TangoConfig.KEY_INT_COLORISO, 100);

        TangoCameraIntrinsics cameraSpecs = mTango.getCameraIntrinsics(mCameraID);
        mBitmap = Bitmap.createBitmap(cameraSpecs.width,cameraSpecs.height, Bitmap.Config.ARGB_8888);
    }

    @Override
    protected void onPause() {
        super.onPause();

        disconnectTango();
        disconnectTangoCamera();

        /*SplashLayout splash = (SplashLayout) findViewById(R.id.component_splash);
        splash.setVisibility(View.VISIBLE); */

    }

    private void disconnectTango() {

        disconnectTangoCamera();

        try {
            if(mIsTangoServiceConnected) {
                //mTango.disconnectCamera(mCameraID);
                mTango.disconnect();
                mTangoUx.stop();
                mIsTangoServiceConnected = false;
            }

        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
    }

    private void disconnectTangoCamera() {
        try {
            synchronized(cameraLock) {

                if(mIsTangoCameraConnected) {
                    mIsTangoCameraConnected = false;
                    mCameraTexture.setVisibility(View.GONE);
                }
            }
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    @Override
    protected void onResume() {
        super.onResume();

        if(mIsCompressing.get()) {
            mIsCompressing.set(false);

            Toast.makeText(getApplicationContext(), "Done!", Toast.LENGTH_LONG).show();

            return;
        }


        //SplashLayout splash = (SplashLayout) findViewById(R.id.component_splash);
        //splash.setVisibility(View.VISIBLE);

        if (!(mIsTangoServiceConnected || mIsTangoCameraConnected)) {
            startActivityForResult(
                    Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                    Tango.TANGO_INTENT_ACTIVITYCODE);

            startActivityForResult(
                    Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_ADF_LOAD_SAVE),
                    Tango.TANGO_INTENT_ACTIVITYCODE);
        }
        Log.i(TAG, "onResumed");
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            Log.i(TAG, "Triggered");
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission,
                        Toast.LENGTH_LONG).show();
                finish();
                return;
            }

         /* New Handler to start the Menu-Activity
         * and close this Splash-Screen after some seconds.*/
         /*   new Handler().postDelayed(new Runnable() {
                @Override
                public void run() {
                // Create an Intent that will start the Menu-Activity.
                    SplashLayout splash = (SplashLayout) findViewById(R.id.component_splash);
                    splash.setVisibility(View.GONE);

                    connectTango();
                    connectTangoCamera();

                }
            }, 1000); */
            connectTango();
            connectTangoCamera();
    }
    }

    private void connectTango() {

        if(mIsCompressing.get())
            return;

        synchronized(cameraLock) {
            try {
                if(!mIsTangoServiceConnected) {

                    StartParams params = new StartParams();
                    mTangoUx.start(params);

                    setupCamera();
                    setTangoListeners();

                    mCameraTexture.connectToTangoCamera(mTango, mCameraID);
                    mTango.connect(mConfig);
                    setUpExtrinsics();

                    mIsTangoServiceConnected = true;
                }
            } catch (TangoOutOfDateException e) {
                if (mTangoUx != null) {
                    mTangoUx.showTangoOutOfDate();
                }
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(), R.string.TangoError,
                        Toast.LENGTH_SHORT).show();
            } catch (SecurityException e) {
                Toast.makeText(getApplicationContext(),
                        R.string.motiontrackingpermission, Toast.LENGTH_SHORT)
                        .show();
            }
        }
    }

    private void connectTangoCamera() {

        if(mIsCompressing.get())
            return;

        try {
            synchronized(cameraLock) {

                if(!mIsTangoCameraConnected) {
                    mCameraTexture.setVisibility(View.VISIBLE);

                    mIsTangoCameraConnected = true;
                }
            }
            //mCameraTexture.setSurfaceTextureListener(this);

        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            //case R.id.first_person_button:
                //mRenderer.setFirstPersonView();
            //    break;
            //case R.id.third_person_button:
                //mRenderer.setThirdPersonView();
            //    break;
            case R.id.top_down_button:
                //mRenderer.setTopDownView();
                synchronized (cameraLock) {
                    mTango.disconnectCamera(mCameraID);
                    //mTango.disconnect();

                    if (mCameraID == TangoCameraIntrinsics.TANGO_CAMERA_COLOR)
                        mCameraID = TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE;
                    else {
                        mCameraID = TangoCameraIntrinsics.TANGO_CAMERA_COLOR;
                    }

                    setupCamera();
                    //mTango.connect(mConfig);
                    mCameraTexture.connectToTangoCamera(mTango,mCameraID);
                }

                break;
            case R.id.take_snap_button:
                takeSnapshot_ButtonClicked();
                break;
            default:
                Log.w(TAG, "Unrecognized button click.");
                break;
        }
    }

    /* @Override
    public boolean onTouchEvent(MotionEvent event) {

        return mRenderer.onTouchEvent(event);
    }*/

    private void setUpExtrinsics() {
        // Set device to imu matrix in Model Matrix Calculator.
        TangoPoseData device2IMUPose = new TangoPoseData();
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        try {
            device2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }

        mRenderer.getModelMatCalculator().SetDevice2IMUMatrix(
                device2IMUPose.getTranslationAsFloats(),
                device2IMUPose.getRotationAsFloats());

        // Set color camera to imu matrix in Model Matrix Calculator.
        TangoPoseData color2IMUPose = new TangoPoseData();

        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        try {
            color2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetColorCamera2IMUMatrix(
                color2IMUPose.getTranslationAsFloats(),
                color2IMUPose.getRotationAsFloats());

        // Get the Camera2Device transform
        float[] rot_Dev2IMU = device2IMUPose.getRotationAsFloats();
        float[] trans_Dev2IMU = device2IMUPose.getTranslationAsFloats();
        float[] rot_Cam2IMU = color2IMUPose.getRotationAsFloats();
        float[] trans_Cam2IMU = color2IMUPose.getTranslationAsFloats();

        float[] dev2IMU = new float[16];
        Matrix.setIdentityM(dev2IMU, 0);
        dev2IMU = ModelMatCalculator.quaternionMatrixOpenGL(rot_Dev2IMU);
        dev2IMU[12] += trans_Dev2IMU[0];
        dev2IMU[13] += trans_Dev2IMU[1];
        dev2IMU[14] += trans_Dev2IMU[2];

        float[] IMU2dev = new float[16];
        Matrix.setIdentityM(IMU2dev, 0);
        Matrix.invertM(IMU2dev, 0, dev2IMU, 0);

        float[] cam2IMU = new float[16];
        Matrix.setIdentityM(cam2IMU, 0);
        cam2IMU = ModelMatCalculator.quaternionMatrixOpenGL(rot_Cam2IMU);
        cam2IMU[12] += trans_Cam2IMU[0];
        cam2IMU[13] += trans_Cam2IMU[1];
        cam2IMU[14] += trans_Cam2IMU[2];

        cam2dev_Transform = new float[16];
        Matrix.setIdentityM(cam2dev_Transform, 0);
        Matrix.multiplyMM(cam2dev_Transform, 0, IMU2dev, 0, cam2IMU, 0);
    }

    private void setTangoListeners() {
        // Configure the Tango coordinate frame pair
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
 //       framePairs.add(new TangoCoordinateFramePair(
 //               TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
 //               TangoPoseData.COORDINATE_FRAME_DEVICE));
                        // Listen for new Tango data
                        mTango.connectListener(framePairs, new OnTangoUpdateListener() {

                            @Override
                            public void onPoseAvailable(final TangoPoseData pose) {
                                // Passing in the pose data to UX library produce exceptions.
                                //if (pose.targetFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION)
                                //    return;

                                if (mTangoUx != null) {
                                    mTangoUx.updatePoseStatus(pose.statusCode);
                                }

                                // Make sure to have atomic access to Tango Pose Data so that
                                // render loop doesn't interfere while Pose call back is updating
                                // the data.
                                synchronized (poseLock) {
                                    mPose = pose;
                                    // Calculate the delta time from previous pose.
                                    mDeltaTime = (pose.timestamp - mPosePreviousTimeStamp) * SECS_TO_MILLISECS;

                                    //Log.i(TAG, String.format("Pose timestamp: %.6f", pose.timestamp));
                                    //Log.i(TAG, String.format("Pose delta: %.2f", mDeltaTime));

                                    mPosePreviousTimeStamp = pose.timestamp;
                                    if (mPreviousPoseStatus != pose.statusCode) {
                                        count = 0;
                                        mValidPoseCallbackCount = 0;
                                    }
                                    mValidPoseCallbackCount++;
                                    count++;
                                    mPreviousPoseStatus = pose.statusCode;
                                    if (mRenderer.isValid() && pose.statusCode == TangoPoseData.POSE_VALID) {
                                        mRenderer.getModelMatCalculator().updateModelMatrix(
                                                pose.getTranslationAsFloats(),
                                                pose.getRotationAsFloats());
                                        mRenderer.updateViewMatrix();
                                    }

                                    // My pose buffering
                                    if (mIsRecording.get() && pose.statusCode == TangoPoseData.POSE_VALID) {
                                        mPosePositionBuffer.add(mNumPoseInSequence, pose.getTranslationAsFloats());
                                        mPoseOrientationBuffer.add(mNumPoseInSequence, pose.getRotationAsFloats());
                                        mPoseTimestampBuffer.add(mNumPoseInSequence, -(float) pose.timestamp);
                                        mNumPoseInSequence++;
                                    }

                                    //End of My pose buffering

                                    poseLock.notifyAll();
                                }
                            }

                            @Override
                            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                                if (mTangoUx != null) {
                                    mTangoUx.updateXyzCount(xyzIj.xyzCount);
                                }

                                // Make sure to have atomic access to TangoXyzIjData so that
                                // render loop doesn't interfere while onXYZijAvailable callback is updating
                                // the point cloud data.

                                TangoPoseData pointCloudPose = new TangoPoseData();
                                pointCloudPose.statusCode = TangoPoseData.POSE_UNKNOWN;

                                synchronized (depthLock) {
                                    mCurrentTimeStamp = xyzIj.timestamp;
                                    mPointCloudFrameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp) * SECS_TO_MILLISECS;
                                    mXyIjPreviousTimeStamp = mCurrentTimeStamp;
                                    mXyzIjCallbackCount++;

                                    try {
                                        pointCloudPose = mTango.getPoseAtTime(mCurrentTimeStamp, framePairs.get(0));
                                        mPointCount = xyzIj.xyzCount;
                                        if (mRenderer.isValid() && pointCloudPose.statusCode == TangoPoseData.POSE_VALID) {
                                            mRenderer.getPointCloud().UpdatePoints(xyzIj.xyz);
                                            mRenderer.getModelMatCalculator().updatePointCloudModelMatrix(
                                                    pointCloudPose.getTranslationAsFloats(),
                                                    pointCloudPose.getRotationAsFloats());
                                            mRenderer.getPointCloud().setModelMatrix(
                                                    mRenderer.getModelMatCalculator().getPointCloudModelMatrixCopy());
                                        }
                                    } catch (TangoErrorException e) {
                                        Toast.makeText(getApplicationContext(), R.string.TangoError,
                                                Toast.LENGTH_SHORT).show();
                                    } catch (TangoInvalidException e) {
                                        Toast.makeText(getApplicationContext(), R.string.TangoError,
                                                Toast.LENGTH_SHORT).show();
                                    }
                                    depthLock.notifyAll();
                                }
                                // My writing to file function

                                if (!mIsRecording.get() || !mPointCloudReady.get() || !mBitmapReady.get()) {
                                    return;
                                }

                                mPointCloudReady.set(false);

                                final TangoPoseData pose = pointCloudPose;

                                final ByteBuffer pointsBuffer = ByteBuffer.allocateDirect(xyzIj.xyzCount * 3 * 4);

                                final int pointCount;
                                final float timestamp;

                                synchronized (depthLock) {
                                    pointCount = xyzIj.xyzCount;
                                    timestamp = (float) xyzIj.timestamp;

                                    pointsBuffer.order(ByteOrder.LITTLE_ENDIAN);
                                    pointsBuffer.asFloatBuffer().put((FloatBuffer) xyzIj.xyz.rewind());

                                    depthLock.notifyAll();
                                }

                                // Background task for writing to file
                                class SendCommandTask extends AsyncTask<Void, Void, Boolean> {
                                    /** The system calls this to perform work in a worker thread and
                                     * delivers it the parameters given to AsyncTask.execute() */
                                    @Override
                                    protected Boolean doInBackground(Void... params) {

                                        try {
                                            mutex_on_mIsRecording.acquire();
                                        } catch (InterruptedException e) {
                                            e.printStackTrace();
                                        }

                                        // Saving the frame or not, depending on the current mode.
                                        //if ( mTimeToTakeSnap || ( mIsRecording && mAutoMode && mXyzIjCallbackCount % 3 == 0 ) ) {
                                        if (mTimeToTakeSnap || (mIsRecording.get() && mAutoMode)) {
                                            //if (mTimeToTakeSnap || (mIsRecording && mAutoMode && mXyzIjCallbackCount % 2 == 0)) {
                                            //writePointCloudToFile(xyzIj, buffer, framePairs);

                                            if (mBitmapReady.get()) {
                                                mNumberOfFrames = mNumberOfFilesWritten;
                                                mRenderer.saveFrame();
                                            }

                                            synchronized (poseLock) {
                                                if (pose.statusCode == TangoPoseData.POSE_VALID) {
                                                    mPosePositionBuffer.add(mNumPoseInSequence, pose.getTranslationAsFloats());
                                                    mPoseOrientationBuffer.add(mNumPoseInSequence, pose.getRotationAsFloats());
                                                    mPoseTimestampBuffer.add(mNumPoseInSequence, -(float) pose.timestamp);
                                                    mNumPoseInSequence++;
                                                }
                                                poseLock.notifyAll();
                                                //End of My pose buffering
                                            }

                                            writePointCloudToFile(pointsBuffer, pointCount, timestamp);
                                        }
                                        mutex_on_mIsRecording.release();

                                        mPointCloudReady.set(true);
                                        return true;
                                    }

                                    /** The system calls this to perform work in the UI thread and delivers
                                     * the result from doInBackground() */
                                    @Override
                                    protected void onPostExecute(Boolean done) {

                                    }
                                }
                                new SendCommandTask().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);

                                // End of My writing to file function
                            }

                            @Override
                            public void onTangoEvent(final TangoEvent event) {
                                if (mTangoUx != null) {
                                    mTangoUx.updateTangoEvent(event);
                                }
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        mTangoEventTextView.setText(event.eventKey + ": " + event.eventValue);
                                    }
                                });
                            }

                            @Override
                            public void onFrameAvailable(int cameraId) {

                                if (cameraId == mCameraID) {

                                    //mGLView.requestRender();

                                    synchronized (cameraLock) {
                                        //Log.i(TAG, "onFrameAvailable: "+ mIsTangoCameraConnected +"\n");
                                        if (mCameraTexture.isAvailable() && mPose != null && mPose.statusCode == TangoPoseData.POSE_VALID
                                                && mIsTangoCameraConnected && mRenderer.frameReady.get()) {
                                            mCameraTexture.onFrameAvailable();
                                        }

                                        cameraLock.notifyAll();
                                    }

                                    //if (!mRenderer.frameReady.get() || !mBitmapReady.get()) {
                                    if (mIsRecording.get() && mRenderer.saveNextFrame.get() && mBitmapReady.get()) {
                                        saveBitmap();
                                    }
                                }
                            }
                        });
    }

    private void saveBitmap() {

        mRenderer.frameReady.set(false);
        mBitmapReady.set(false);
        mRenderer.saveNextFrame.set(false);

        // Background task for writing to file
        class SendCommandTask extends AsyncTask<Void, Void, Boolean> {
            /** The system calls this to perform work in a worker thread and
             * delivers it the parameters given to AsyncTask.execute() */
            @Override
            protected Boolean doInBackground(Void... params) {

                // Saving the frame or not, depending on the current mode.
                //if ( mTimeToTakeSnap || ( mIsRecording && mAutoMode && mXyzIjCallbackCount % 3 == 0 ) ) {
                //if (mTimeToTakeSnap || (mIsRecording && mAutoMode)) {

                mRenderer.frameReady.set(false);

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        synchronized (colorLock) {
                            synchronized (cameraLock)
                            {
                                mCameraTexture.getBitmap(mBitmap);
                                mCameraTime = mCameraTexture.getTimestamp();
                            }
                            mRenderer.frameReady.set(true);
                            colorLock.notifyAll();
                        }
                    }
                });

                mBitmapReady.set(false);

                try {
                    synchronized (colorLock) {
                        if (!mRenderer.frameReady.get())
                            colorLock.wait(500);

                        colorLock.notifyAll();
                    }

                    // Get pose
                    final TangoPoseData cameraPose = mTango.getPoseAtTime(mCameraTime, new TangoCoordinateFramePair(
                            TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                            TangoPoseData.COORDINATE_FRAME_DEVICE));

                    final int poseNumber;
                    synchronized (poseLock) {
                        poseNumber = mNumPoseInSequence;
                        if (cameraPose.statusCode == TangoPoseData.POSE_VALID) {
                            mPosePositionBuffer.add(mNumPoseInSequence, cameraPose.getTranslationAsFloats());
                            mPoseOrientationBuffer.add(mNumPoseInSequence, cameraPose.getRotationAsFloats());
                            mPoseTimestampBuffer.add(mNumPoseInSequence, (float) -cameraPose.timestamp);
                            mNumPoseInSequence++;
                        }
                        poseLock.notifyAll();
                        //End of My pose buffering
                    }

                    // Create the capture file.
                    String bitmapFileName = "";
                    if (mCameraID == TangoCameraIntrinsics.TANGO_CAMERA_COLOR)
                        bitmapFileName = "pc_" + mNowTimeString + "_" + String.format("%04d_%05d.jpg", mNumberOfFrames, poseNumber);
                    else if (mCameraID == TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE)
                        bitmapFileName = "pc_" + mNowTimeString + "_" + String.format("%04d_%05d.pgm", mNumberOfFrames, poseNumber);

                    File bitmapFile = new File(mSaveDirAbsPath, bitmapFileName);

                    FileOutputStream fileOutputStream = new FileOutputStream(bitmapFile);
                    BufferedOutputStream bos = new BufferedOutputStream(fileOutputStream);
                    //FileChannel fileChannel = fileOutputStream.getChannel();

                    // Bitmap conveniently provides file output.
                    synchronized (colorLock) {
                        //Bitmap bitmap = Bitmap.createBitmap(mRenderer.pixels, mRenderer.offscreenSize_.x, mRenderer.offscreenSize_.y, Bitmap.Config.ARGB_8888);

                        //ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
                        //bitmap.compress(Bitmap.CompressFormat.JPEG, 100, byteStream);

                        //ByteBuffer byteBuffer = ByteBuffer.wrap(byteStream.toByteArray());
                        //fileChannel.write(byteBuffer);

                        if (mCameraID == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                            mBitmap.compress(Bitmap.CompressFormat.JPEG, 100, bos);

                            bos.flush();
                            bos.close();

                        } else if (mCameraID == TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE) {
                                        /*Bitmap alphaBitmap = Bitmap.createBitmap(mBitmap.getWidth(), mBitmap.getHeight(),
                                                        Bitmap.Config.ALPHA_8);
                                        float[] matrix = new float[] {
                                                0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0,
                                                0.2989f, 0.5870f, 0.1140f, 0, 0};
                                        Paint grayToAlpha = new Paint();
                                        grayToAlpha.setColorFilter(new ColorMatrixColorFilter(new ColorMatrix(matrix)));
                                        Canvas alphaCanvas = new Canvas(alphaBitmap);
                                        // Make sure nothing gets scaled during drawing
                                        alphaCanvas.setDensity(Bitmap.DENSITY_NONE);
                                        // Draw grayscale bitmap on to alpha canvas, using color filter that
                                        // takes alpha from red channel
                                        alphaCanvas.drawBitmap(mBitmap, 0, 0, grayToAlpha);*/
                            //alpha.compress(Bitmap.CompressFormat.PNG, 100, bos);

                            //PGM
                            DataOutputStream out = new DataOutputStream(bos);
                            int width = mBitmap.getWidth();
                            int height = mBitmap.getHeight();

                            out.writeBytes("P5\n");
                            out.writeBytes("#created by Google Tango " + String.format("%.6f", mCameraTime) + "\n");
                            out.writeBytes(width + " " + height + "\n255\n");

                            final byte[] pixelBytes = new byte[width * height];
                            final int[] pixels = new int[width * height];
                            mBitmap.getPixels(pixels, 0, width, 0, 0, width, height);

                            for (int i = 0; i < height * width; i++) {
                                pixelBytes[i] = (byte) (pixels[i] >> 8); //Green
                            }

                            out.write(pixelBytes);
                            out.close();

                            //mBitmap.compress(Bitmap.CompressFormat.JPEG, 100, bos);
                        }

                        //fileChannel.close();
                        fileOutputStream.close();

                        //mBitmap.recycle();

                        colorLock.notifyAll();
                    }

                    mBitmapReady.set(true);

                    synchronized (fileLock) {
                        mFilenameBuffer.add(mSaveDirAbsPath + bitmapFileName);
                        fileLock.notifyAll();
                    }

                } catch (Exception e) {
                    e.printStackTrace();
                }
                //}

                //sendBroadcast(new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE, Uri.fromFile(bitmapFile)));
                mBitmapReady.set(true);

                return true;
            }

            /** The system calls this to perform work in the UI thread and delivers
             * the result from doInBackground() */
            @Override
            protected void onPostExecute(Boolean done) {

            }
        }
        new SendCommandTask().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
    }

    /**
     * Create a separate thread to update Log information on UI at the specified interval of
     * UPDATE_INTERVAL_MS. This function also makes sure to have access to the mPose atomically.
     */
    private void startUIThread() {
        new Thread(new Runnable() {
            final DecimalFormat threeDec = new DecimalFormat("0.000");

            @Override
            public void run() {
                while (true) {
                    try {
                        Thread.sleep(UPDATE_INTERVAL_MS);
                        // Update the UI with TangoPose information
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {

                                int poseStatus;
                                String translationString;
                                String quaternionString;
                                String deltaTimeString;
                                String countString;

                                String pointCountString;
                                String pointDeltaString;

                                synchronized (poseLock) {
                                    if (mPose == null || !mIsTangoCameraConnected || !mIsTangoServiceConnected) {
                                        return;
                                    }

                                    translationString = "["
                                            + threeDec.format(mPose.translation[0]) + ", "
                                            + threeDec.format(mPose.translation[1]) + ", "
                                            + threeDec.format(mPose.translation[2]) + "] ";
                                    quaternionString = "["
                                            + threeDec.format(mPose.rotation[0]) + ", "
                                            + threeDec.format(mPose.rotation[1]) + ", "
                                            + threeDec.format(mPose.rotation[2]) + ", "
                                            + threeDec.format(mPose.rotation[3]) + "] ";

                                    deltaTimeString = threeDec.format(mDeltaTime);

                                    countString = Integer.toString(count);

                                    poseStatus = mPose.statusCode;

                                    poseLock.notifyAll();
                                }

                                synchronized (depthLock) {

                                        // Display number of points in the point cloud
                                    pointCountString = Integer.toString(mPointCount);

                                    pointDeltaString = threeDec.format(mPointCloudFrameDelta);

                                    depthLock.notifyAll();
                                }

                                // My GUI updates
                                // Display a waiting progress bar
                                        /*if (mAppIsStarting && mPose.statusCode == TangoPoseData.POSE_VALID ) {
                                            mWaitingLinearLayout.setVisibility(View.GONE);
                                            mAppIsStarting = false;
                                        }*/

                                synchronized (cameraLock){
                                    // Display pose data on screen in TextViews
                                    mPoseTextView.setText(translationString);
                                    mQuatTextView.setText(quaternionString);
                                    mPoseCountTextView.setText(countString);
                                    mDeltaTextView.setText(deltaTimeString);

                                    if (poseStatus == TangoPoseData.POSE_VALID) {
                                        mPoseStatusTextView.setText(R.string.pose_valid);
                                    } else if (poseStatus == TangoPoseData.POSE_INVALID) {
                                        mPoseStatusTextView.setText(R.string.pose_invalid);
                                    } else if (poseStatus == TangoPoseData.POSE_INITIALIZING) {
                                        mPoseStatusTextView.setText(R.string.pose_initializing);
                                    } else if (poseStatus == TangoPoseData.POSE_UNKNOWN) {
                                        mPoseStatusTextView.setText(R.string.pose_unknown);
                                    }

                                    mPointCountTextView.setText(pointCountString);
                                    mFrequencyTextView.setText(pointDeltaString);
                                        /*mAverageZTextView.setText(""
                                                + threeDec.format(mRenderer.getPointCloud()
                                                .getAverageZ()));*/

                                    mFilesWrittenToSDCardTextView.setText("" +
                                            String.valueOf(mNumberOfFilesWritten) + "\n" + mFilename);
                                }
                                // End of My GUI updates
                            }
                        });
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                }
        }).start();
    }

    // My functions

    // This function is called when the Take Snapshot button is clicked
    private void takeSnapshot_ButtonClicked() {
        mTimeToTakeSnap=true;
    }

    // This function is called when the Auto Mode Switch is changed
    private void autoMode_SwitchChanged(boolean isChecked) {
        mAutoMode = isChecked;
    }

    // This function is called when the Record Switch is changed
    private void record_SwitchChanged(boolean isChecked) {
        try {
            mutex_on_mIsRecording.acquire();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Start Recording
        if (isChecked) {
            // Generate a new date number to create a new group of files
            /*Calendar rightNow = Calendar.getInstance();
            int hour = rightNow.get(Calendar.HOUR_OF_DAY);
            int minute = rightNow.get(Calendar.MINUTE);
            int sec = rightNow.get(Calendar.SECOND);
            int milliSec = rightNow.get(Calendar.MILLISECOND);
            mNowTimeString = "" + (int)(1000000 * hour + 10000 * minute + 100 * sec + (float)milliSec / 10.0);*/

            String dateFormat = "yyyyMMdd_kkmmss";
            DateFormat formatter = new SimpleDateFormat(dateFormat);
            mNowTimeString = formatter.format(Calendar.getInstance().getTime());

            mNumberOfFilesWritten = 0;
            // Enable snapshot button
            mTakeSnapButton.setEnabled(true);

            mIsRecording.set(true);
        }
        // Finish Recording
        else {
            mIsRecording.set(false);
            mIsCompressing.set(true);

            // Disable snapshot button
            mTakeSnapButton.setEnabled(false);

            /*SplashLayout splash = (SplashLayout) findViewById(R.id.component_splash);
            splash.setVisibility(View.VISIBLE);*/

            // Display a waiting progress bar
            mWaitingLinearLayout.setVisibility(View.VISIBLE);
            mWaitingTextView.setText(R.string.waitSavingScan);

            disconnectTangoCamera();

            // Background task for writing poses to file
            class SendCommandTask extends AsyncTask<Context, Void, Uri> {
                /** The system calls this to perform work in a worker thread and
                 * delivers it the parameters given to AsyncTask.execute() */
                @Override
                protected Uri doInBackground(Context... myAppContext) {

                    synchronized (colorLock){
                        if (!mBitmapReady.get())
                            try{
                                colorLock.wait(1000);
                            }
                            catch (InterruptedException ex){}
                    }

                    synchronized (poseLock) {
                        int numPoints = mNumPoseInSequence;

                        for (int i = 0; i < numPoints; i++) {

                            try{
                                final TangoPoseData areaPose = mTango.getPoseAtTime(-mPoseTimestampBuffer.get(i),new TangoCoordinateFramePair(
                                        TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION, TangoPoseData.COORDINATE_FRAME_DEVICE));

                                if (areaPose.statusCode == TangoPoseData.POSE_VALID) {
                                    mPosePositionBuffer.add(mNumPoseInSequence, areaPose.getTranslationAsFloats());
                                    mPoseOrientationBuffer.add(mNumPoseInSequence, areaPose.getRotationAsFloats());
                                    mPoseTimestampBuffer.add(mNumPoseInSequence, (float) areaPose.timestamp);
                                    mNumPoseInSequence++;
                                }
                            }
                            catch (TangoInvalidException e) {
                                Log.e(TAG, e.toString());
                            }
                        }
                    }

                    disconnectTango();

                    // Stop the Pose Recording, and write them to a file.
                    writePoseToFile(mNumPoseInSequence);

                    // If a snap has been asked just before, but not saved, ignore it, otherwise,
                    // it will be saved at the end dof this function, and the 2nd archive will override
                    // the first.
                    mTimeToTakeSnap = false;
                    mNumPoseInSequence = 0;
                    mPoseOrientationBuffer.clear();
                    mPoseOrientationBuffer.clear();
                    mPoseTimestampBuffer.clear();

                    String zipFilename;

                    synchronized (fileLock) {
                        // Zip all the files from this sequence
                        zipFilename = mSaveDirAbsPath + "TangoData_" + mNowTimeString +
                                "_" + mFilenameBuffer.size() + "files.zip";
                        String[] fileList = mFilenameBuffer.toArray(new String[mFilenameBuffer.size()]);
                        ZipWriter zipper = new ZipWriter(fileList, zipFilename);
                        zipper.zip();

                        // Delete the data files now that they are archived
                        for (String s : mFilenameBuffer) {
                            File file = new File(s);
                            boolean deleted = file.delete();
                            if (!deleted) {
                                Log.w(TAG, "File \"" + s + "\" not deleted\n");
                            }
                        }
                        mFilenameBuffer.clear();
                    }

                    // Send the zip file to another app
                    File myZipFile = new File(zipFilename);

                    return FileProvider.getUriForFile(myAppContext[0], "com.kitware." +
                            "tangoproject.paraviewtangorecorder.fileprovider", myZipFile);
                }

                /** The system calls this to perform work in the UI thread and delivers
                 * the result from doInBackground() */
                @Override
                protected void onPostExecute(Uri fileURI) {

                    /*SplashLayout splash = (SplashLayout) findViewById(R.id.component_splash);
                    splash.setVisibility(View.VISIBLE);*/

                    Intent shareIntent = new Intent();
                    shareIntent.setAction(Intent.ACTION_SEND);
                    shareIntent.putExtra(Intent.EXTRA_STREAM, fileURI);
                    shareIntent.setType("application/zip");
                    startActivity(Intent.createChooser(shareIntent, "Send Scan To..."));
                    mWaitingLinearLayout.setVisibility(View.GONE);

                    // Make the new file visible to other apps.
                    try {
                        sendBroadcast(new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE, fileURI));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }


                //mIsCompressing.set(false);
                    //connectTango();

                    //synchronized(cameraLock) {
                        //mCameraTexture.connectToTangoCamera(mTango,mCameraID);
                        //mIsTangoCameraConnected = true;
                    //}
                }
            }
            new SendCommandTask().execute(this);

        }

        mutex_on_mIsRecording.release();

    }

    // This function writes the XYZ points to .vtk files in binary
    private void writePointCloudToFile(ByteBuffer pointsBuffer, int pointCount, float timestamp) {

        mFilename = "pc_" + mNowTimeString + "_" + String.format("%04d", mNumberOfFilesWritten) +
                ".vtk";

        synchronized (fileLock) {
            mFilenameBuffer.add(mSaveDirAbsPath + mFilename);
        }
        File file = new File(dir, mFilename);

        try {

            //DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
            //                         new FileOutputStream(file)));

            FileOutputStream outFile = new FileOutputStream(file);
            FileChannel outChannel = outFile.getChannel();

            /*out.write(("# vtk DataFile Version 3.0\n" +
                    "vtk output\n" +
                    "BINARY\n" +
                    "DATASET POLYDATA\n" +
                    "POINTS " + xyzIj.xyzCount + " float\n").getBytes());*/


            final ByteBuffer pointsHeader = ByteBuffer.wrap(("# vtk DataFile Version 3.0\n" +
                    "vtk output\n" +
                    "BINARY\n" +
                    "DATASET POLYDATA\n" +
                    "POINTS " + pointCount + " float\n").getBytes());


            /*ByteBuffer myBuffer = ByteBuffer.allocate(xyzIj.xyzCount * 3 * 4);
            myBuffer.asFloatBuffer().put(xyzIj.xyz);

            for (int i = 0; i < xyzIj.xyzCount; i++) {

                out.writeFloat(myBuffer.getFloat(3 * i * 4));
                out.writeFloat(myBuffer.getFloat((3 * i + 1) * 4));
                out.writeFloat(myBuffer.getFloat((3 * i + 2) * 4));
            }*/

            /*out.write(("\nVERTICES 1 " + String.valueOf(xyzIj.xyzCount + 1) + "\n").getBytes());
            out.writeInt(xyzIj.xyzCount);
            for (int i = 0; i < xyzIj.xyzCount; i++) {
                out.writeInt(i);
            }*/

            /*ByteBuffer verticeHeader = ByteBuffer.wrap(("\nVERTICES 1 " + String.valueOf(xyzIj.xyzCount + 1) + "\n").getBytes());

            int [] vertices = new int[1 + xyzIj.xyzCount];
            vertices[0] = xyzIj.xyzCount;
            for (int i = 0; i < xyzIj.xyzCount; i++) {
                vertices[i+1] = i;
            }

            ByteBuffer verticeBuffer = ByteBuffer.allocate(vertices.length * 4);
            verticeBuffer.order(ByteOrder.LITTLE_ENDIAN);
            IntBuffer intBuffer = verticeBuffer.asIntBuffer();
            intBuffer.put(vertices);*/

            //out.write(("\nFIELD FieldData 1\n" + "timestamp 1 1 float\n").getBytes());
            //out.writeFloat((float) xyzIj.timestamp);

            final ByteBuffer timeHeader = ByteBuffer.wrap(("\nFIELD FieldData 1\n" + "timestamp 1 1 float\n").getBytes());

            final ByteBuffer timeBuffer = ByteBuffer.allocateDirect(4);
            timeBuffer.order(ByteOrder.LITTLE_ENDIAN);
            timeBuffer.asFloatBuffer().put(timestamp);

            pointsHeader.rewind();
            pointsBuffer.rewind();
            timeHeader.rewind();
            timeBuffer.rewind();

            final ByteBuffer [] outputBuffers = new ByteBuffer[]{pointsHeader, pointsBuffer, timeHeader, timeBuffer};
            outChannel.write(outputBuffers);

            outChannel.close();
            outFile.close();

            //out.close();

            mNumberOfFilesWritten++;
            mTimeToTakeSnap = false;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // This function writes the pose data and timestamps to .vtk files in binary
    private void writePoseToFile(int numPoints) {

        String poseFileName = "pc_" + mNowTimeString + "_poses.vtk";
        synchronized (fileLock) {
            mFilenameBuffer.add(mSaveDirAbsPath + poseFileName);
        }
        File file = new File(dir, poseFileName);

        try {
            DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
                    new FileOutputStream(file)));

            out.write(("# vtk DataFile Version 3.0\n" +
                    "vtk output\n" +
                    "BINARY\n" +
                    "DATASET POLYDATA\n" +
                    "POINTS " + numPoints + " float\n").getBytes());

            for (int i = 0; i < numPoints; i++) {
                out.writeFloat(mPosePositionBuffer.get(i)[0]);
                out.writeFloat(mPosePositionBuffer.get(i)[1]);
                out.writeFloat(mPosePositionBuffer.get(i)[2]);
            }

            out.write(("\nLINES 1 " + String.valueOf(numPoints + 1) + "\n").getBytes());
            out.writeInt(numPoints);
            for (int i = 0; i < numPoints; i++) {
                out.writeInt(i);
            }

            out.write(("\nFIELD FieldData 1\n" +
                    "Cam2Dev_transform 16 1 float\n").getBytes());
            for (int i = 0; i < cam2dev_Transform.length; i++) {
                out.writeFloat(cam2dev_Transform[i]);
            }

            out.write(("\nPOINT_DATA " + String.valueOf(numPoints) + "\n" +
                    "FIELD FieldData 2\n" +
                    "orientation 4 " + String.valueOf(numPoints) + " float\n").getBytes());

            for (int i = 0; i < numPoints; i++) {
                out.writeFloat(mPoseOrientationBuffer.get(i)[0]);
                out.writeFloat(mPoseOrientationBuffer.get(i)[1]);
                out.writeFloat(mPoseOrientationBuffer.get(i)[2]);
                out.writeFloat(mPoseOrientationBuffer.get(i)[3]);
            }

            out.write(("\ntimestamp 1 " + String.valueOf(numPoints) + " float\n").getBytes());
            for (int i = 0; i < numPoints; i++) {
                out.writeFloat(mPoseTimestampBuffer.get(i));
            }

            out.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /*public synchronized void attachTexture(final int cameraId, final int textureName) {
        if (textureName > 0) {
            // Link the texture with Tango if the texture changes after
            // Tango is connected. This generally doesn't happen but
            // technically could because they happen in separate
            // threads. Otherwise the link will be made in startTango().
            mTango.connectTextureId(cameraId, textureName);

        }
    }

    public Point getCameraFrameSize(int cameraId) {
        TangoCameraIntrinsics intrinsics = mTango.getCameraIntrinsics(cameraId);
        return new Point(intrinsics.width, intrinsics.height);
    }

    public void updateTexture(int cameraId) {
            try {
                synchronized (colorLock)
                {
                    mTango.updateTexture(cameraId);
                }
            }
            catch (TangoInvalidException e) {
                e.printStackTrace();
            }
    }*/
    // End of My functions
}
