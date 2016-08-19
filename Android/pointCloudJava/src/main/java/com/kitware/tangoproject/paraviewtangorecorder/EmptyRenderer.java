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
 */

package com.kitware.tangoproject.paraviewtangorecorder;

import android.view.MotionEvent;

import com.projecttango.tangoutils.Renderer;
import com.projecttango.tangoutils.renderables.CameraFrustum;
import com.projecttango.tangoutils.renderables.CameraFrustumAndAxis;
import com.projecttango.tangoutils.renderables.Grid;
import com.projecttango.tangoutils.renderables.PointCloud;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * OpenGL rendering class for the Motion Tracking API sample. This class
 * managers the objects visible in the OpenGL view which are the
 * {@link CameraFrustum}, {@link PointCloud} and the {@link Grid}. These objects
 * are implemented in the TangoUtils library in the package
 * {@link com.projecttango.tangoutils.renderables}.
 *
 * This class receives {@link TangoPose} data from the {@link MotionTracking}
 * class and updates the model and view matrices of the {@link Renderable}
 * objects appropriately. It also handles the user-selected camera view, which
 * can be 1st person, 3rd person, or top-down.
 *
 */
public class EmptyRenderer extends Renderer {

    private PointCloud mPointCloud;
    private Grid mGrid;
    private CameraFrustumAndAxis mCameraFrustumAndAxis;
    private static boolean mIsValid = false;

    AtomicBoolean saveNextFrame = new AtomicBoolean(false);
    AtomicBoolean frameReady = new AtomicBoolean(true);

    public EmptyRenderer() {
    }

    public PointCloud getPointCloud() {
        return mPointCloud;
    }

    public boolean isValid(){
        return mIsValid;
    }

    public void saveFrame() {
        saveNextFrame.set(true);
    }

    //@Override
    //public boolean onTouchEvent(MotionEvent event) {return true;}
}