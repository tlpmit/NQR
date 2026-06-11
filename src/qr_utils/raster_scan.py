import numpy as np
import math

######################################
# raster version of depth map
######################################

tiny = 1.0e-6

# Should be in-line...
def edgeFunction(a, b, c):
    return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])

########
# laserScanParams = (0.3, 0.3, 0.4, 0.1, 5., 50)
# (focal, height, width, near, length, n) = robot.laserScanParams[cam]
# Raster(width, height, 2*n, 2*n, near, length, focal)
# viewPort = [0, rasterGlobal[cam].imageWidth, 0, rasterGlobal[cam].imageHeight,
#                        0.0, 0.1]
# wm.makeWindow('Raster_'+cam, viewPort, 2*n+10)
########

class Raster:
    def __init__(self, sensor_params):
        (fov_deg_width, imageHeight, imageWidth,  nearClippingPlane, farClippingPlane) = sensor_params
        imageHeight /= 8
        imageWidth /= 8
        screenWidth = 0.3
        ratio = imageHeight/imageWidth
        screenHeight = ratio * screenWidth
        focalLength = screenWidth/(2*np.tan(np.deg2rad(fov_deg_width/2)))
        self.screenWidth = screenWidth
        self.screenHeight = screenHeight
        self.imageWidth = int(imageWidth)
        self.imageHeight = int(imageHeight)
        self.nearClippingPlane = nearClippingPlane
        self.farClippingPlane = farClippingPlane
        self.focalLength = focalLength
        screenAspectRatio = screenWidth/screenHeight
        imageAspectRatio = imageWidth / imageHeight
        top = ((screenHeight/2.) / focalLength) * nearClippingPlane
        right = ((screenWidth/2.) / focalLength) * nearClippingPlane
        # field of view (horizontal)
        xscale = 1.; yscale = 1.
        # fov = 2*180/math.pi*math.atan((screenWidth / 2.) / focalLength)
        # print('fov=', fov)
        if screenAspectRatio > imageAspectRatio:
            xscale = imageAspectRatio / screenAspectRatio
        else:
            yscale = screenAspectRatio / imageAspectRatio
        right *= xscale
        top *= yscale
        bottom = -top
        left = -right
        self.screenCoordinates = (left, right, top, bottom)
        size = int(imageWidth * imageHeight)
        self.depthBuffer = np.full(size, farClippingPlane, dtype=np.float64)
        self.frameBuffer = np.zeros(size, dtype=np.int32)
        self.saved_depthBuffer = None
        self.saved_frameBuffer = None

    def revert(self):
        assert self.saved_depthBuffer is not None and self.saved_frameBuffer is not None
        np.copyto(self.depthBuffer, self.saved_depthBuffer)
        np.copyto(self.frameBuffer, self.saved_frameBuffer)

    def save(self):
        if self.saved_depthBuffer is None:
            self.saved_depthBuffer = self.depthBuffer.copy()
        else:
            np.copyto(self.saved_depthBuffer, self.depthBuffer)
        if self.saved_frameBuffer is None:
            self.saved_frameBuffer = self.frameBuffer.copy()
        else:
            np.copyto(self.saved_frameBuffer, self.frameBuffer)

    def reset(self):
        self.depthBuffer.fill(self.farClippingPlane)
        self.frameBuffer.fill(0.)
        self.saved_depthBuffer = None
        self.saved_frameBuffer = None

    def convertToRaster(self, vertexCamera, vertexRaster):
        (l, r, t, b) = self.screenCoordinates
        vScreen_x = self.nearClippingPlane * vertexCamera[0] / -vertexCamera[2]
        vScreen_y = self.nearClippingPlane * vertexCamera[1] / -vertexCamera[2]
        # from screen to NDC (range [-1,1])
        vNDC_x = 2. * vScreen_x / (r - l) - (r + l) / (r - l)
        vNDC_y = 2. * vScreen_y / (t - b) - (t + b) / (t - b)
        vertexRaster[0] = (vNDC_x + 1) / 2 * self.imageWidth
        # in raster space y is down
        vertexRaster[1] = (1 - vNDC_y) / 2 * self.imageHeight
        vertexRaster[2] = - vertexCamera[2]

    def update(self, mesh, objId):
        assert objId > 0, 'Raster update, objId <= 0'
        v0r = np.zeros(3, dtype=np.float64)
        v1r = np.zeros(3, dtype=np.float64)
        v2r = np.zeros(3, dtype=np.float64)
        pix = np.zeros(2, dtype=np.float64)

        verts = mesh.vertices
        # Remember that z points AWAY from the scene, so -z increases with depth.
        zmin = verts.min(axis=0)[2]
        # Punt on meshs behind the eye
        if -zmin < self.nearClippingPlane  or -zmin > self.farClippingPlane:
            return
        # if zmax > min_depth+tiny:
        #     Plane = np.array([0., 0., 1, -min_depth])
        #     # TODO: Implement by finding vertices above Plane and doing convex hull
        #     below, above = facePrimCut(Plane, prim)
        #     if not above:
        #         return
        #     if above is not mesh:
        #         self.update(above, objId, onlyUpdate=onlyUpdate, ignoreNear=ignoreNear)
        #         return
        faces = mesh.faces
        for face_i in range(faces.shape[0]):
            # face is array of indices for verts in face
            v0 = verts[faces[face_i,0], :]
            v1 = verts[faces[face_i,1], :]
            v2 = verts[faces[face_i,2], :]            
            # Convert the vertices of the triangle to raster space
            self.convertToRaster(v0, v0r)
            self.convertToRaster(v1, v1r)
            self.convertToRaster(v2, v2r)

            # Precompute reciprocal of vertex z-coordinate
            v0r[2] = 1. / v0r[2]
            v1r[2] = 1. / v1r[2]
            v2r[2] = 1. / v2r[2]

            xmin = min(v0r[0], v1r[0], v2r[0])
            ymin = min(v0r[1], v1r[1], v2r[1])
            xmax = max(v0r[0], v1r[0], v2r[0])
            ymax = max(v0r[1], v1r[1], v2r[1])

            # the triangle is out of screen
            if (xmin > self.imageWidth - 1 or xmax < 0 \
                or ymin > self.imageHeight - 1 or ymax < 0): continue

            # be careful xmin/xmax/ymin/ymax can be negative
            x0 = int(max(0, math.floor(xmin)))
            x1 = int(min(self.imageWidth - 1, math.floor(xmax)))
            y0 = int(max(0, math.floor(ymin)))
            y1 = int(min(self.imageHeight - 1, math.floor(ymax)))

            area = edgeFunction(v0r, v1r, v2r)

            # Inner loop
            dbuff = self.depthBuffer
            fbuff = self.frameBuffer
            for y in range(y0, y1+1):
                for x in range(x0, x1+1):
                    pix[0] = x + 0.5; pix[1] = y + 0.5
                    # w0 = edgeFunction(v1r, v2r, pix);
                    w0 = (pix[0] - v1r[0]) * (v2r[1] - v1r[1]) - (pix[1] - v1r[1]) * (v2r[0] - v1r[0])
                    # w1 = edgeFunction(v2r, v0r, pix);
                    w1 = (pix[0] - v2r[0]) * (v0r[1] - v2r[1]) - (pix[1] - v2r[1]) * (v0r[0] - v2r[0])
                    # w2 = edgeFunction(v0r, v1r, pix);
                    w2 = (pix[0] - v0r[0]) * (v1r[1] - v0r[1]) - (pix[1] - v0r[1]) * (v1r[0] - v0r[0])
                    if (w0 >= 0 and w1 >= 0 and w2 >= 0):
                        w0 /= area;
                        w1 /= area;
                        w2 /= area;
                        oneOverZ = v0r[2] * w0 + v1r[2] * w1 + v2r[2] * w2;
                        z = 1. / oneOverZ;
                        # Depth-buffer test
                        off = y * self.imageWidth + x
                        if (z < dbuff[off]):
                            dbuff[off] = z
                            fbuff[off] = objId

    def countId(self, objId):
        fbuff = self.frameBuffer
        # dbuff = self.depthBuffer
        # near = 0 if ignoreNear else self.nearClippingPlane
        # return np.count_nonzero(np.logical_and(fbuff == objId, dbuff > near))
        return np.count_nonzero(fbuff == objId)
    
    def countIdVsSaved(self, objId, saved_objId):
        fbuff = self.frameBuffer
        saved = self.saved_frameBuffer
        # dbuff = self.depthBuffer
        # near = 0 if ignoreNear else self.nearClippingPlane
        # return np.count_nonzero(np.logical_and(fbuff == objId, dbuff > near))
        return np.count_nonzero((fbuff == objId) & (saved == saved_objId))


