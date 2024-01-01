/*
    Copyright 2013-2022 Will Winder

    This file is part of Universal Gcode Sender (UGS).

    UGS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    UGS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with UGS.  If not, see <http://www.gnu.org/licenses/>.
 */
package com.willwinder.ugs.nbm.visualizer.shared;

import com.willwinder.ugs.nbm.visualizer.options.VisualizerOptions;
import com.willwinder.ugs.nbm.visualizer.renderables.*;
import com.willwinder.ugs.nbp.lib.lookup.CentralLookup;
import com.willwinder.universalgcodesender.i18n.Localization;
import com.willwinder.universalgcodesender.model.BackendAPI;
import com.willwinder.universalgcodesender.model.Position;
import com.willwinder.universalgcodesender.model.UnitUtils;
import com.willwinder.universalgcodesender.model.events.SettingChangedEvent;
import com.willwinder.universalgcodesender.uielements.helpers.FPSCounter;
import com.willwinder.universalgcodesender.uielements.helpers.Overlay;
import com.willwinder.universalgcodesender.utils.Settings;
import com.willwinder.universalgcodesender.visualizer.MouseProjectionUtils;
import com.willwinder.universalgcodesender.visualizer.VisualizerUtils;

import org.lwjgl.opengl.awt.AWTGLCanvas;
import org.lwjgl.opengl.awt.GLData;
import org.joml.Matrix4f;
import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;
import org.openide.util.lookup.ServiceProvider;
import org.openide.util.lookup.ServiceProviders;

import java.awt.*;
import java.awt.event.InputEvent;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.Collections;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import static com.willwinder.ugs.nbm.visualizer.options.VisualizerOptions.VISUALIZER_OPTION_BG;
import javax.swing.SwingUtilities;

/**
 * 3D Canvas for GCode Visualizer
 *
 * @author wwinder
 */
@SuppressWarnings("serial")
@ServiceProviders(value = {
        @ServiceProvider(service = IRenderableRegistrationService.class),
        @ServiceProvider(service = GcodeRenderer.class) })
public class GcodeRenderer extends AWTGLCanvas implements IRenderableRegistrationService {
    private static final Logger logger = Logger.getLogger(GcodeRenderer.class.getName());

    private static boolean ortho = true;
    private static boolean debugCoordinates = false; // turn on coordinate debug output

    // Machine data
    private final Position machineCoord;
    private final Position workCoord;

    // Projection variables
    private Position center;
    private Position eye;
    private Position objectMin;
    private Position objectMax;
    private double maxSide;
    private int xSize;
    private int ySize;
    private double minArcLength;
    private double arcLength;

    // Scaling
    private double scaleFactor = 1;
    private double scaleFactorBase = 1;
    private double zoomMultiplier = 1;
    private boolean invertZoom = false;
    // const values until added to settings
    private final double minZoomMultiplier = .1;
    private final double maxZoomMultiplier = 50;
    private final double zoomIncrement = 0.2;

    // Movement
    private final int panMouseButton = InputEvent.BUTTON2_MASK; // TODO: Make configurable
    private double panMultiplierX = 1;
    private double panMultiplierY = 1;
    private Position translationVectorH;
    private Position translationVectorV;

    // Mouse rotation data
    private Point mouseLastWindow;
    private Point mouseCurrentWindow;
    private Position mouseWorldXY;
    private Position rotation;

    private FPSCounter fpsCounter;
    private Overlay overlay;
    private final String dimensionsLabel = "";

    private final java.util.List<Renderable> objects;
    private boolean idle = true;

    // Preferences
    private java.awt.Color clearColor;

    /**
     * Constructor.
     */
    public GcodeRenderer() {
        super(getGLConfiguration());

        eye = new Position(0, 0, 1.5);
        center = new Position(0, 0, 0);
        objectMin = new Position(-10, -10, -10);
        objectMax = new Position(10, 10, 10);

        workCoord = new Position(0, 0, 0);
        machineCoord = new Position(0, 0, 0);

        rotation = new Position(0.0, -30.0, 0.0);
        setVerticalTranslationVector();
        setHorizontalTranslationVector();

        objects = new CopyOnWriteArrayList<>();
        objects.add(new MachineBoundries(Localization.getString("platform.visualizer.renderable.machine-boundries")));
        objects.add(new Tool(Localization.getString("platform.visualizer.renderable.tool-location")));
        objects.add(new MouseOver(Localization.getString("platform.visualizer.renderable.mouse-indicator")));
        objects.add(
                new OrientationCube(0.5f, Localization.getString("platform.visualizer.renderable.orientation-cube")));
        objects.add(new Grid(Localization.getString("platform.visualizer.renderable.grid")));
        Collections.sort(objects);

        reloadPreferences();
        listenForSettingsEvents();
    }

    private static GLData getGLConfiguration() {
        GLData data = new GLData();
//        data.majorVersion = 2;
//        data.minorVersion = 1;

//        data.contextReleaseBehavior = GLData.ReleaseBehavior.FLUSH;
         data.samples = 8;
        return data;
    }

    private void listenForSettingsEvents() {
        BackendAPI backendAPI = CentralLookup.getDefault().lookup(BackendAPI.class);
        Settings settings = backendAPI.getSettings();
        invertZoom = settings.isInvertMouseZoom();

        backendAPI.addUGSEventListener(event -> {
            if (event instanceof SettingChangedEvent) {
                invertZoom = backendAPI.getSettings().isInvertMouseZoom();
            }
        });
    }

    @Override
    public final Collection<Renderable> getRenderables() {
        return objects;
    }

    @Override
    public void registerRenderable(Renderable r) {
        if (r == null) return;
        if (!objects.contains(r)) {
            objects.add(r);
            Collections.sort(objects);
        }
    }

    @Override
    public void removeRenderable(Renderable r) {
        if (r == null) return;
        if (objects.contains(r)) {
            objects.remove(r);
            Collections.sort(objects);
        }
    }

    /**
     * Get the location on the XY plane of the mouse.
     */
    public Position getMouseWorldLocation() {
        return this.mouseWorldXY;
    }

    public void setWorkCoordinate(Position p) {
        if (p != null) {
            this.workCoord.set(p.getPositionIn(UnitUtils.Units.MM));
        }
    }

    public void setMachineCoordinate(Position p) {
        if (p != null) {
            this.machineCoord.set(p.getPositionIn(UnitUtils.Units.MM));
        }
    }

    final public void reloadPreferences() {
        VisualizerOptions vo = new VisualizerOptions();

        clearColor = vo.getOptionForKey(VISUALIZER_OPTION_BG).value;

        for (Renderable r : objects) {
            r.reloadPreferences(vo);
        }
    }

    // ------ Implement methods declared in GLEventListener ------

    /**
     * Called back immediately after the OpenGL context is initialized. Can be used
     * to perform one-time initialization. Run only once.
     * GLEventListener method.
     */
    @Override
    public void initGL() {
        logger.log(Level.INFO, "Initializing OpenGL context.");
        // TODO: Figure out scale factor / dimensions label based on GcodeRenderer
        /*
            this.scaleFactorBase = VisualizerUtils.findScaleFactor(this.xSize, this.ySize, this.objectMin, this.objectMax);
            this.scaleFactor = this.scaleFactorBase * this.zoomMultiplier;

            double objectWidth = this.objectMax.x-this.objectMin.x;
            double objectHeight = this.objectMax.y-this.objectMin.y;
            this.dimensionsLabel = Localization.getString("VisualizerCanvas.dimensions") + ": " 
                    + Localization.getString("VisualizerCanvas.width") + "=" + format.format(objectWidth) + " " 
                    + Localization.getString("VisualizerCanvas.height") + "=" + format.format(objectHeight);

        */
        
        GL.createCapabilities();

        this.fpsCounter = new FPSCounter(new Font("SansSerif", Font.BOLD, 12));
        this.overlay = new Overlay(new Font("SansSerif", Font.BOLD, 12));
        this.overlay.setColor(127, 127, 127, 100);
        this.overlay.setTextLocation(Overlay.LOWER_LEFT);

        // Parse random gcode file and generate something to draw.
        GL11.glShadeModel(GL11.GL_SMOOTH); // blends colors nicely, and smoothes out lighting
        GL11.glClearColor(clearColor.getRed() / 255f, clearColor.getGreen() / 255f, clearColor.getBlue() / 255f,
                clearColor.getAlpha() / 255f);
        GL11.glClearDepth(1.0f); // set clear depth value to farthest
        GL11.glEnable(GL11.GL_BLEND);
        GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA);
        GL11.glEnable(GL11.GL_DEPTH_TEST);
        GL11.glDepthFunc(GL11.GL_LEQUAL); // the type of depth test to do
        GL11.glHint(GL11.GL_PERSPECTIVE_CORRECTION_HINT, GL11.GL_NICEST); // best perspective correction

        /*
         * GL11.glLoadIdentity();
         * float[] lmodel_ambient = { 0.5f, 0.5f, 0.5f, 1.0f };
         * GL11.glLightModelfv(GL11.GL_LIGHT_MODEL_AMBIENT, lmodel_ambient, 0);
         */

        // init lighting
        float[] ambient = { .6f, .6f, .6f, 1.f };
        float[] diffuse = { .6f, .6f, .6f, 1.0f };
        float[] position = { 0f, 0f, 20f, 1.0f };

        GL11.glLightfv(GL11.GL_LIGHT0, GL11.GL_AMBIENT, ambient);
        GL11.glLightfv(GL11.GL_LIGHT0, GL11.GL_DIFFUSE, diffuse);
        GL11.glEnable(GL11.GL_LIGHT0);
        GL11.glLightfv(GL11.GL_LIGHT0, GL11.GL_POSITION, position);

        // Allow glColor to set colors
        GL11.glEnable(GL11.GL_COLOR_MATERIAL);
        GL11.glColorMaterial(GL11.GL_FRONT, GL11.GL_DIFFUSE);
        GL11.glColorMaterial(GL11.GL_FRONT, GL11.GL_AMBIENT);
        // GL11.glColorMaterial(GL11.GL_FRONT_AND_BACK, GL11.GL_AMBIENT_AND_DIFFUSE);
        // GL11.glColorMaterial(GL11.GL_FRONT, GL11.GL_SPECULAR);

        float diffuseMaterial[] = { 0.5f, 0.5f, 0.5f, 1.0f };

        GL11.glMaterialfv(GL11.GL_FRONT, GL11.GL_DIFFUSE, diffuseMaterial);
        // GL11.glMaterialfv(GL11.GL_FRONT, GL11.GL_SPECULAR, mat_specular, 0);
        // GL11.glMaterialf(GL11.GL_FRONT, GL11.GL_SHININESS, 25.0f);

        // GL11.glMaterialfv(GL11.GL_FRONT_AND_BACK, GL11.GL_AMBIENT_AND_DIFFUSE);

        GL11.glEnable(GL11.GL_LIGHTING);
        for (Renderable r : objects) {
            r.init();
        }
    }
    
    public void onResize(int x, int y, int width, int height) {
        // logger.log(Level.INFO, "Reshaping OpenGL context.");
        this.xSize = width;
        this.ySize = height;

        resizeForCamera(objectMin, objectMax, 0.9);
    }

    public void setObjectSize(Position min, Position max) {
        if (min == null || max == null) {
            this.objectMin = new Position(-10, -10, -10);
            this.objectMax = new Position(10, 10, 10);
            idle = true;
        } else {
            this.objectMin = min;
            this.objectMax = max;
            idle = false;
        }
        resizeForCamera(objectMin, objectMax, 0.9);
    }

    /**
     * Zoom the visualizer to the given region.
     */
    public void zoomToRegion(Position min, Position max, double bufferFactor) {
        if (min == null || max == null)
            return;

        if (this.ySize == 0) {
            this.ySize = 1;
        } // prevent divide by zero

        // Figure out offset compared to the current center.
        Position regionCenter = VisualizerUtils.findCenter(min, max);
        this.eye.x = regionCenter.x - this.center.x;
        this.eye.y = regionCenter.y - this.center.y;

        // Figure out what the scale factors would be if we reset this object.
        double _scaleFactorBase = VisualizerUtils.findScaleFactor(this.xSize, this.ySize, min, max, bufferFactor);
        double _scaleFactor = _scaleFactorBase * this.zoomMultiplier;

        // Calculate the zoomMultiplier needed to get to that scale, and set it.
        this.zoomMultiplier = _scaleFactor / this.scaleFactorBase;
        this.scaleFactor = this.scaleFactorBase * this.zoomMultiplier;
    }

    /**
     * Zoom to display the given region leaving the suggested buffer.
     */
    private void resizeForCamera(Position min, Position max, double bufferFactor) {
        if (min == null || max == null)
            return;

        if (this.ySize == 0) {
            this.ySize = 1;
        } // prevent divide by zero

        this.center = VisualizerUtils.findCenter(min, max);
        this.scaleFactorBase = VisualizerUtils.findScaleFactor(this.xSize, this.ySize, min, max, bufferFactor);
        this.scaleFactor = this.scaleFactorBase * this.zoomMultiplier;
        this.panMultiplierX = VisualizerUtils.getRelativeMovementMultiplier(min.x, max.x, this.xSize);
        this.panMultiplierY = VisualizerUtils.getRelativeMovementMultiplier(min.y, max.y, this.ySize);
    }

    /**
     * Called back by the animator to perform rendering.
     * GLEventListener method.
     */
    @Override
    public void paintGL() {
        // needs lwjgl3-awt 1.9.0 to be published
//        int w = getFramebufferWidth();
//        int h = getFramebufferHeight();
        int w = xSize;
        int h = ySize;
        if (w == 0 || h == 0) {
            return;
        }

        this.setupPerpective(this.xSize, this.ySize, ortho);

        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
        GL11.glViewport(0, 0, w, h);

        // Update normals when an object is scaled
        GL11.glEnable(GL11.GL_NORMALIZE);

        // Setup the current matrix so that the projection can be done.
        if (mouseLastWindow != null) {
            GL11.glPushMatrix();
            GL11.glRotated(this.rotation.x, 0.0, 1.0, 0.0);
            GL11.glRotated(this.rotation.y, 1.0, 0.0, 0.0);
            GL11.glTranslated(-this.eye.x - this.center.x, -this.eye.y - this.center.y, -this.eye.z - this.center.z);
            this.mouseWorldXY = MouseProjectionUtils.intersectPointWithXYPlane(
                    mouseLastWindow.x, mouseLastWindow.y);
            GL11.glPopMatrix();
        } else {
            this.mouseWorldXY = new Position(0, 0, 0);
        }

        // Render the different parts of the scene.
        for (Renderable r : objects) {
            // Don't draw disabled renderables.
            if (!r.isEnabled())
                continue;

            GL11.glPushMatrix();
            // in case a renderable sets the color, set it back to gray and opaque.
            GL11.glColor4f(0.5f, 0.5f, 0.5f, 1f);

            if (r.rotate()) {
                GL11.glRotated(this.rotation.x, 0.0, 1.0, 0.0);
                GL11.glRotated(this.rotation.y, 1.0, 0.0, 0.0);
            }
            if (r.center()) {
                GL11.glTranslated(-this.eye.x - this.center.x, -this.eye.y - this.center.y,
                        -this.eye.z - this.center.z);
            }

            if (!r.enableLighting()) {
                GL11.glDisable(GL11.GL_LIGHTING);
            }
            try {
                r.draw(idle, machineCoord, workCoord, objectMin, objectMax, scaleFactor, mouseWorldXY,
                        rotation);
            } catch (Exception e) {
                logger.log(Level.SEVERE, "An exception occurred while drawing " + r.getClass().getSimpleName(), e);
            }
            if (!r.enableLighting()) {
                GL11.glEnable(GL11.GL_LIGHTING);
                GL11.glEnable(GL11.GL_LIGHT0);
            }
            GL11.glPopMatrix();
        }

        this.fpsCounter.draw();
        this.overlay.draw(this.dimensionsLabel);

        GL11.glLoadIdentity();
        swapBuffers();

        update();
    }
    
    @Override
    public void repaint() {
        if (SwingUtilities.isEventDispatchThread()) {
            render();
        } else {
            SwingUtilities.invokeLater(this::render);
        }
    }

    /**
     * Setup the perspective matrix.
     */
    private void setupPerpective(int x, int y, boolean ortho) {
        float aspectRatio = (float) x / y;

        if (ortho) {
            GL11.glMatrixMode(GL11.GL_PROJECTION);
            GL11.glLoadIdentity();
            GL11.glOrtho(-0.60 * aspectRatio / scaleFactor, 0.60 * aspectRatio / scaleFactor, -0.60 / scaleFactor,
                    0.60 / scaleFactor,
                    -10 / scaleFactor, 10 / scaleFactor);
            GL11.glMatrixMode(GL11.GL_MODELVIEW);
            GL11.glLoadIdentity();
        } else {
            Matrix4f m = new Matrix4f();
            // Setup perspective projection, with aspect ratio matches viewport
            m.perspective((float) Math.toRadians(45), aspectRatio, 0.1f, 20000.0f);
            // Move camera out and point it at the origin
            m.lookAt((float) this.eye.x, (float) this.eye.y, (float) this.eye.z,
                    0, 0, 0,
                    0, 1, 0);

            FloatBuffer fb = BufferUtils.createFloatBuffer(16);
            m.get(fb);

            GL11.glMatrixMode(GL11.GL_PROJECTION);
            GL11.glLoadMatrixf(fb);

            // Enable the model-view transform
            GL11.glMatrixMode(GL11.GL_MODELVIEW);
            GL11.glLoadIdentity(); // reset
        }
    }

    /**
     * Called after each render.
     */
    private void update() {
        if (debugCoordinates) {
            System.out.println("Machine coordinates: " + this.machineCoord.toString());
            System.out.println("Work coordinates: " + this.workCoord.toString());
            System.out.println("-----------------");
        }
    }

    /**
     * Called back before the OpenGL context is destroyed.
     * Release resource such as buffers.
     * GLEventListener method.
     */
    @Override
    synchronized public void disposeCanvas() {
        super.disposeCanvas();
        logger.log(Level.INFO, "Disposing OpenGL context.");
    }

    private void setHorizontalTranslationVector() {
        double x = Math.cos(Math.toRadians(this.rotation.x));
        double xz = Math.sin(Math.toRadians(this.rotation.x));

        double y = xz * Math.sin(Math.toRadians(this.rotation.y));
        double yz = xz * Math.cos(Math.toRadians(this.rotation.y));

        translationVectorH = new Position(x, y, yz);
        translationVectorH.normalizeXYZ();
    }

    private void setVerticalTranslationVector() {
        double y = Math.cos(Math.toRadians(this.rotation.y));
        double yz = Math.sin(Math.toRadians(this.rotation.y));

        translationVectorV = new Position(0, y, yz);
        translationVectorV.normalizeXYZ();
    }

    public void mouseMoved(Point lastPoint) {
        mouseLastWindow = lastPoint;        
    }

    public void mouseRotate(Point point) {
        this.mouseCurrentWindow = point;
        if (this.mouseLastWindow != null) {
            int dx = this.mouseCurrentWindow.x - this.mouseLastWindow.x;
            int dy = this.mouseCurrentWindow.y - this.mouseLastWindow.y;

            rotation.x += dx / 2.0;
            rotation.y = Math.min(0, Math.max(-180, this.rotation.y += dy / 2.0));

            if (ortho) {
                setHorizontalTranslationVector();
                setVerticalTranslationVector();
            }
        }

        // Now that the motion has been accumulated, reset last.
        this.mouseLastWindow = this.mouseCurrentWindow;
    }

    public void mousePan(Point point) {
        this.mouseCurrentWindow = point;
        int dx = this.mouseCurrentWindow.x - this.mouseLastWindow.x;
        int dy = this.mouseCurrentWindow.y - this.mouseLastWindow.y;
        pan(dx, dy);
    }

    public void pan(int dx, int dy) {
        if (ortho) {
            // Treat dx and dy as vectors relative to the rotation angle.
            this.eye.x -= ((dx * this.translationVectorH.x * this.panMultiplierX) + (dy * this.translationVectorV.x * panMultiplierY));
            this.eye.y += ((dy * this.translationVectorV.y * panMultiplierY) - (dx * this.translationVectorH.y * this.panMultiplierX));
            this.eye.z -= ((dx * this.translationVectorH.z * this.panMultiplierX) + (dy * this.translationVectorV.z * panMultiplierY));
        } else {
            this.eye.x += dx;
            this.eye.y += dy;
        }

        // Now that the motion has been accumulated, reset last.
        this.mouseLastWindow = this.mouseCurrentWindow;
    }

    public void zoom(int delta) {
        if (delta == 0)
            return;

        if (delta > 0) {
            if (this.invertZoom)
                zoomOut(delta);
            else
                zoomIn(delta);
        } else if (delta < 0) {
            if (this.invertZoom)
                zoomIn(delta * -1);
            else
                zoomOut(delta * -1);
        }
    }

    private void zoomOut(int increments) {
        if (ortho) {
            if (this.zoomMultiplier <= this.minZoomMultiplier)
                return;

            this.zoomMultiplier -= increments * zoomIncrement;
            if (this.zoomMultiplier < this.minZoomMultiplier)
                this.zoomMultiplier = this.minZoomMultiplier;

            this.scaleFactor = this.scaleFactorBase * this.zoomMultiplier;
        } else {
            this.eye.z += increments;
        }
    }

    private void zoomIn(int increments) {
        if (ortho) {
            if (this.zoomMultiplier >= this.maxZoomMultiplier)
                return;

            this.zoomMultiplier += increments * zoomIncrement;
            if (this.zoomMultiplier > this.maxZoomMultiplier)
                this.zoomMultiplier = this.maxZoomMultiplier;

            this.scaleFactor = this.scaleFactorBase * this.zoomMultiplier;
        } else {
            this.eye.z -= increments;
        }
    }

    /**
     * Reset the view angle and zoom.
     */
    public void resetView() {
        moveCamera(new Position(0, 0, 1.5), new Position(0, -30, 0), 1);
    }

    /**
     * Moves the camera to a position and rotation
     *
     * @param position to the given position
     * @param rotation directs the camera given this rotation
     * @param zoom     the zoom level
     */
    public void moveCamera(Position position, Position rotation, double zoom) {
        this.zoomMultiplier = Math.min(Math.max(zoom, minZoomMultiplier), maxZoomMultiplier);
        this.scaleFactor = this.scaleFactorBase;
        this.eye = new Position(position);
        this.rotation = new Position(rotation);
    }
}
