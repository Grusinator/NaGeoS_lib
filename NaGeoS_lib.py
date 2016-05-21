# -*- coding: utf-8 -*-
"""
Created on Wed May 11 13:43:44 2016

@author: grusinator

Nageos Lib
"""
import os
import gdal,ogr
import numpy as np

class NaGeoS_lib():
    def __init__(self,datapath):
        #test if path exist - if not create it
        def testpath(path):
            if not os.path.exists(path):
                os.mkdir(path)
                print 'new directory created: ' + path
        
        #root data dir
        self.datapath = datapath
        testpath(self.datapath)
#==============================================================================
#       Vector init       
#==============================================================================
        #vectorpath init
        self.vectorpath = self.datapath + '/Objects/'
        testpath(self.vectorpath)

        #fetch vector file list
        VectorNameList = self.GetVectorList()
        self.Vector = dict()
        
        #Read the vector files that are available
        for Name in VectorNameList:        
            self.Vector[Name] = Vectordata(self.vectorpath + Name) 
            
#==============================================================================
#       Raster init
#==============================================================================
        #rasterpath init
        self.rasterpath = self.datapath + '/DSM/'
        testpath(self.rasterpath)
        
        self.Terrainfn = '/DSM'

        #Initialize Raster terrain file
        self.RasterDSM = Rasterdata(self.rasterpath + self.Terrainfn)  
#==============================================================================
# general data functions
#==============================================================================
    def GetVectorList(self):
        VectorNameList = ['Bebyggelse','Lufthavn','Veje','BackU','FootBallU']
        return VectorNameList
        
    def GetRasterList(self): # not used unless more DSM's are used
        RasterNameList = ['DSM']
        return RasterNameList    
#==============================================================================
# Raster functions
#==============================================================================
    def GetVerticalDistanceToDSM(self,position):
        dist = self.RasterDSM.GetVerticalDistanceTo(position)
        return dist


#==============================================================================
# Vector functions
#==============================================================================
    def GetMinDistanceToObject(self,Object,position):
        min_dist = self.Vector[Object].VectorGetDistanceTo(position)
        return min_dist
        
            #update statusfile
    def ParseVectorData(self,Name,Data):
        Vectorfn = Name + '.shp'
        self.Vector[Name] = Vectordata(Vectorfn)

#==============================================================================
# RasterClass
#==============================================================================
class Rasterdata():
    def __init__(self,datapath):
        self.datapath = datapath + '.tif'
        self.data = gdal.Open(self.datapath)
        
        self.transform = self.data.GetGeoTransform()
        
        self.pixelWidth = self.transform[1]
        self.pixelHeight = self.transform[5]
        
        self.cols = self.data.RasterXSize
        self.rows = self.data.RasterYSize
        
        #raster corners
        self.xLeft = self.transform[0]
        self.yTop = self.transform[3]
        self.xRight = self.xLeft+self.cols*self.pixelWidth
        self.yBottom = self.yTop-self.rows*self.pixelHeight
        
        self.extent = self.GetRasterExtent()

        
    def GetPath(self):
        return self.datapath
        
    def GetVerticalDistanceTo(self,position):
        #if self.TestPositionWithinRaster(position): 
        xpos = np.round((position.GetX()-self.xLeft)/self.pixelWidth).astype(np.int)
        ypos = np.round((position.GetY()-self.yTop)/self.pixelHeight).astype(np.int)
        print xpos 
        print ypos
        dataarray = self.data.GetRasterBand(1).ReadAsArray(xpos,ypos,1,1).astype(np.float)
        dist = position.GetZ()-dataarray
        #else:
       #    dist = None
        return dist
        
    def GetRasterExtent(self):
        # Get raster geometry
        ring = ogr.Geometry(ogr.wkbMultiPolygon)
        ring.AddPoint(self.xLeft,  self.yTop)
        ring.AddPoint(self.xLeft,  self.yBottom)
        ring.AddPoint(self.xRight, self.yTop)
        ring.AddPoint(self.xRight, self.yBottom)
        ring.AddPoint(self.xLeft,  self.yTop)
        rasterGeometry = ogr.Geometry(ogr.wkbPolygon)
        rasterGeometry.AddGeometry(ring)
        
        return rasterGeometry
        
    def TestPositionWithinRaster(self,position):
        print self.extent.Intersect(position)
        return self.extent.Contains(position)
                        
            
#==============================================================================
# Vector Class   
#==============================================================================
class Vectordata():
    def __init__(self,datapath):
        self.datapath = datapath + '.shp'
        
        self.data = ogr.Open(self.datapath)
        self.layer = self.data.GetLayer()
        self.GeomType = self.layer.GetGeomType()
        
    def GetPath(self):
        return self.datapath
    
    def VectorGetDistanceTo(self,position):
        
        bufferdist = 10000
        #self.layer.SetSpatialFilter(position.Buffer(bufferdist))
        #it only takes the intersect
        
        featcount = self.layer.GetFeatureCount()
        if featcount == 0:
            min_dist = None
        else:
            dist = np.ones((featcount))*bufferdist
            for i in range(featcount):
                feat = self.layer.GetFeature(i)
                geom = feat.geometry()
                dist[i] = ogr.Geometry.Distance(position,geom)
            min_dist = np.min(dist)
            
        return min_dist
        
#==============================================================================
# Start script
#==============================================================================
datapath = '/home/grusinator/NaGeoS_Data/'
                  
nageos = NaGeoS_lib(datapath)
print nageos.Vector['Veje'].GetPath()

point = ogr.Geometry(ogr.wkbPoint)
point.AddPoint(720920.638,6187734.508,100)

print nageos.GetMinDistanceToObject('Veje',point)

print nageos.GetVerticalDistanceToDSM(point)


