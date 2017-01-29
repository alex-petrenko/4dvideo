package com.threedvideo.threedvideograbber;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoCameraPreview;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.google.atap.tangoservice.experimental.TangoImageBuffer;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;


public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";

    private Tango tango;
    private TangoConfig tangoConfig;
    private TangoCameraPreview tangoCameraPreview = null;
    private TangoPointCloudManager pointCloudManager;

    // Java filesystem API is kinda messy, so I ended up having all those entities :)
    private FileOutputStream captureFileStream = null;
    private DataOutputStream captureDataStream = null;
    private FileChannel captureFileChannel = null;

    private double lastCapturedPointCloudTimestamp = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    @Override
    protected void onStart() {
        super.onStart();

        tango = new Tango(MainActivity.this, new Runnable() {
            @Override
            public void run() {
                synchronized (MainActivity.this) {
                    try {
                        TangoSupport.initialize();

                        tangoConfig = setupTangoConfig(tango);
                        tango.connect(tangoConfig);

                        tangoCameraPreview = (TangoCameraPreview) findViewById(R.id.tango_camera_preview);
                        tangoCameraPreview.connectToTangoCamera(tango, TangoCameraIntrinsics.TANGO_CAMERA_COLOR);

                        pointCloudManager = new TangoPointCloudManager();

                        String filename = "/sdcard/3dvideo/file.bin";  // hardcoded path, okay for a demo
                        captureFileStream = new FileOutputStream(filename);
                        captureDataStream = new DataOutputStream(captureFileStream);
                        captureFileChannel = captureFileStream.getChannel();

                        startupTango();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, "Tango service out of date", e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Tango service error", e);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, "Tango invalid exception", e);
                    } catch (IOException e) {
                        Log.e(TAG, "IO exception", e);
                        e.printStackTrace();
                    }
                }
            }
        });
    }

    @Override
    protected void onPause() {
        super.onPause();

        synchronized (MainActivity.this) {
            if (captureFileStream != null) {
                try {
                    captureFileStream.close();
                } catch (IOException e) {
                    Log.e(TAG, "IO exception", e);
                    e.printStackTrace();
                }
            }

            if (tangoCameraPreview != null)
                tangoCameraPreview.disconnectFromTangoCamera();
            tango.disconnect();
        }
    }

    private TangoConfig setupTangoConfig(Tango t) {
        TangoConfig config = t.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);

        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_HIGH_RATE_POSE, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_SMOOTH_POSE, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);

        return config;
    }

    private void startupTango() {
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<>();
        framePairs.add(new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE, TangoPoseData.COORDINATE_FRAME_DEVICE));
        tango.connectListener(framePairs, new Tango.OnTangoUpdateListener() {
            @Override
            public void onPoseAvailable(TangoPoseData tangoPoseData) {
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData tangoXyzIjData) {
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR)
                    tangoCameraPreview.onFrameAvailable();
            }

            @Override
            public void onTangoEvent(TangoEvent tangoEvent) {
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData tangoPointCloudData) {
                Log.d(TAG, String.format("Num points %d", tangoPointCloudData.numPoints));
                pointCloudManager.updatePointCloud(tangoPointCloudData);
            }
        });

        tango.experimentalConnectOnFrameListener(TangoCameraIntrinsics.TANGO_CAMERA_COLOR, new Tango.OnFrameAvailableListener() {
            @Override
            public void onFrameAvailable(TangoImageBuffer tangoImageBuffer, int id) {
                if (tangoImageBuffer == null) {
                    Log.d(TAG, "Image buffer is null!");
                    return;
                }

                TangoPointCloudData pointCloud = pointCloudManager.getLatestPointCloud();
                if (pointCloud == null) {
                    Log.d(TAG, "Point cloud is null!");
                    return;
                }

                TangoPoseData pose = TangoSupport.calculateRelativePose(
                        tangoImageBuffer.timestamp,
                        TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR,
                        pointCloud.timestamp,
                        TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH
                );

                if (captureFileStream != null) {
                    try {
                        captureDataStream.writeDouble(tangoImageBuffer.timestamp);

                        long startTime = System.currentTimeMillis();
                        captureFileChannel.write(tangoImageBuffer.data);
                        Log.d(TAG, String.format("Wrote image data to file in %d ms", System.currentTimeMillis() - startTime));
                        Log.d(TAG, String.format("Image info: %dx%d, fmt:%d size:%d", tangoImageBuffer.width, tangoImageBuffer.height, tangoImageBuffer.format, tangoImageBuffer.data.position(), tangoImageBuffer.data.limit()));

                        final int numPoints = lastCapturedPointCloudTimestamp == pointCloud.timestamp ? 0 : pointCloud.numPoints;
                        lastCapturedPointCloudTimestamp = pointCloud.timestamp;
                        captureDataStream.writeDouble(lastCapturedPointCloudTimestamp);
                        captureDataStream.write(numPoints);

                        if (numPoints > 0) {
                            startTime = System.currentTimeMillis();
                            ByteBuffer byteBuffer = ByteBuffer.allocate(pointCloud.points.capacity() * 4);
                            byteBuffer.asFloatBuffer().put(pointCloud.points);
                            captureFileChannel.write(byteBuffer);
                            Log.d(TAG, String.format("Wrote points data to file in %d ms", System.currentTimeMillis() - startTime));
                        }

                        float[] rotation = pose.getRotationAsFloats();
                        float[] translation = pose.getTranslationAsFloats();
                        for (int i = 0; i < rotation.length; ++i)
                            captureDataStream.writeFloat(rotation[i]);
                        for (int i = 0; i < translation.length; ++i)
                            captureDataStream.writeFloat(translation[i]);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }

                if (pose.statusCode != TangoPoseData.POSE_VALID)
                    Log.d(TAG, String.format("Pose invalid, status %d", pose.statusCode));
            }
        });
    }
}
