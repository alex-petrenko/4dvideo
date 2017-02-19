package com.threedvideo.threedvideograbber;

import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;

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
    private int numFrames = 0;
    private volatile TangoImageBuffer currentImageBuffer;
    private Object mutex = new Object();
    private volatile boolean capturingStarted = false;
    private volatile boolean stopCapturing = false;
    private Thread capturingThread;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    @Override
    protected void onStart() {
        super.onStart();

        Button startButton = (Button) findViewById(R.id.start_capture_button);
        startButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                synchronized (mutex) {
                    capturingStarted = true;
                }
            }
        });

        Button stopButton = (Button) findViewById(R.id.stop_capture_button);
        stopButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                synchronized (mutex) {
                    stopCapturing = true;
                }
            }
        });

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


                        int epochSeconds = (int)(System.currentTimeMillis() / 1000);
                        String filename = "/sdcard/3dvideo/" + Integer.toString(epochSeconds) + "_dataset.bin";
                        captureFileStream = new FileOutputStream(filename);
                        captureDataStream = new DataOutputStream(captureFileStream);
                        captureFileChannel = captureFileStream.getChannel();

                        startupTango();

                        startCapturing();
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

        Log.d(TAG, "OnPause");

        synchronized (mutex) {
            stopCapturing = true;
            mutex.notifyAll();
        }

        if (capturingThread != null)
            try {
                capturingThread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        synchronized (MainActivity.this) {
            if (captureFileStream != null) {
                try {
                    captureFileChannel.close();
                    captureDataStream.close();
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

                synchronized (mutex) {
                    currentImageBuffer = copyImageBuffer(tangoImageBuffer);
                    mutex.notifyAll();
                }
            }
        });
    }

    TangoImageBuffer copyImageBuffer(TangoImageBuffer imageBuffer) {
        ByteBuffer clone = ByteBuffer.allocateDirect(imageBuffer.data.capacity());
        imageBuffer.data.rewind();
        clone.put(imageBuffer.data);
        imageBuffer.data.rewind();
        clone.flip();
        return new TangoImageBuffer(
                imageBuffer.width,
                imageBuffer.height,
                imageBuffer.stride,
                imageBuffer.frameNumber,
                imageBuffer.timestamp,
                imageBuffer.format,
                clone
        );
    }

    private void startCapturing() {
        capturingThread = new Thread() {
            @Override
            public void run() {
                TangoImageBuffer imageBuffer = null;

                while (!stopCapturing) {
                    synchronized (mutex) {
                        try {
                            mutex.wait();

                            if (stopCapturing)
                                break;

                            imageBuffer = copyImageBuffer(currentImageBuffer);
                            Log.d(TAG, "Copied latest image buffer!");
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    if (imageBuffer == null)
                        continue;

                    Log.d(TAG, "Image buffer is not null");

                    if (!capturingStarted)
                        continue;

                    TangoPointCloudData pointCloud = pointCloudManager.getLatestPointCloud();
                    if (pointCloud == null) {
                        Log.d(TAG, "Point cloud is null!");
                        continue;
                    }

                    if (pointCloud.numPoints == 0) {
                        Log.d(TAG, "Point cloud is empty!");
                        continue;
                    }

                    TangoPoseData pose = TangoSupport.calculateRelativePose(
                            imageBuffer.timestamp,
                            TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR,
                            pointCloud.timestamp,
                            TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH
                    );

                    if (captureFileStream != null && pose.statusCode == TangoPoseData.POSE_VALID) {
                        try {
                            int numPoints = lastCapturedPointCloudTimestamp == pointCloud.timestamp ? 0 : pointCloud.numPoints;
                            lastCapturedPointCloudTimestamp = pointCloud.timestamp;
                            if (numPoints > 0) {
                                captureDataStream.writeDouble(imageBuffer.timestamp);

                                long startTime = System.currentTimeMillis();
                                captureFileChannel.write(imageBuffer.data);
                                Log.d(TAG, String.format("Wrote image data to file in %d ms, timestamp %f", System.currentTimeMillis() - startTime, imageBuffer.timestamp));
                                Log.d(TAG, String.format("Image info: %dx%d, fmt: %d, size: %d", imageBuffer.width, imageBuffer.height, imageBuffer.format, imageBuffer.data.position()));

                                captureDataStream.writeDouble(lastCapturedPointCloudTimestamp);
                                captureDataStream.writeInt(numPoints);

                                startTime = System.currentTimeMillis();
                                ByteBuffer byteBuffer = ByteBuffer.allocate(pointCloud.points.capacity() * 4);
                                byteBuffer.asFloatBuffer().put(pointCloud.points);

                                int numBytes = pointCloud.points.capacity() * 4;
                                captureDataStream.writeInt(numBytes);
                                Log.d(TAG, String.format("Num bytes: %d", numBytes));
                                captureFileChannel.write(byteBuffer);
                                Log.d(TAG, String.format("Wrote %d points to file in %d ms, size is %d, buffer capacity %d, buffer pos %d", numPoints, System.currentTimeMillis() - startTime, byteBuffer.position(), pointCloud.points.capacity(), pointCloud.points.position()));

                                float[] rotation = pose.getRotationAsFloats();
                                float[] translation = pose.getTranslationAsFloats();
                                for (int i = 0; i < rotation.length; ++i)
                                    captureDataStream.writeFloat(rotation[i]);
                                for (int i = 0; i < translation.length; ++i)
                                    captureDataStream.writeFloat(translation[i]);

                                ++numFrames;
                                Log.d(TAG, String.format("Total %d frames", numFrames));
                            }
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
                }
            }
        };

        capturingThread.start();
    }
}
