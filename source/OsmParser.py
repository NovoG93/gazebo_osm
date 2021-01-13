#!/usr/bin/env python
import sys
import os
from lxml import etree
import argparse
import utm

from dict2sdf import GetSDF
from osm2dict import Osm2Dict
from getMapImage import getMapImage
from getOsmFile import getOsmFile

class OsmParser:
    def __init__(self):
        self.args = None
    
    def parse_args_(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('-f', '--outFile',
                            help='Output file name', type=str, default='outFile.sdf')
        parser.add_argument('-o', '--osmFile', help='Name of the osm file generated',
                            type=str,
                            default='map.osm')
        parser.add_argument('-O', '--inputOsmFile', help='Name of the Input osm file',
                            type=str,
                            default='')
        parser.add_argument('-d', '--directory',
                            help='Output directory',
                            type=str,
                            default='./')
        parser.add_argument('-B', '--boundingbox',
                            help=('Give the bounding box for the area\n' +
                                'Format: MinLon MinLat MaxLon MaxLat'),
                            nargs='*',
                            type=float)
        parser.add_argument('-r', '--roads',
                            help='Display Roads',
                            action='store_true')

        parser.add_argument('-m', '--models',
                            help='Display models',
                            action='store_true')

        parser.add_argument('-b', '--buildings',
                            help='Display buildings',
                            action='store_true')

        parser.add_argument('-a', '--displayAll',
                            help='Display roads and models',
                            action='store_true')

        self.args = parser.parse_args()

    def parse_osm_(self):
        if not self.args:
            self.parse_args_()
        
        flags = []
        if self.args.buildings:
            flags.append('b')
        if self.args.models:
            flags.append('m')
        if self.args.roads:
            flags.append('r')
        if not(self.args.roads or self.args.models or self.args.buildings) or self.args.displayAll:
            flags.append('a')

        if not os.path.exists(self.args.directory):
            os.makedirs(self.args.directory)

        self.args.osmFile = self.args.directory + '/' + self.args.osmFile
        self.args.outFile = self.args.directory + '/' + self.args.outFile

        osmDictionary = {}

        if self.args.inputOsmFile:
            f = open(self.args.inputOsmFile, 'r')
            root = etree.fromstring(f.read())
            f.close()
            self.args.boundingbox = [root[0].get('minlon'),
                                root[0].get('minlat'),
                                root[0].get('maxlon'),
                                root[0].get('maxlat')]

        else:
            if not self.arg.boundingbox:
                print("Can not get osm file from server without bounding box")
                sys.exit("No bounding box for target area")
            
            print("Downloading the osm data ... ")

        osmDictionary = getOsmFile(self.args.boundingbox,
                                self.args.osmFile, self.args.inputOsmFile)
 

        #Initialize Osm Dictionary class
        osmRoads = Osm2Dict(self.args.boundingbox[0], self.args.boundingbox[1],
                            osmDictionary, flags)

        print("Extracting the map data for gazebo ...")
        #Get Road and model details
        roadPointWidthMap, modelPoseMap, buildingLocationMap = osmRoads.getMapDetails()

        print("Building sdf file ...")
        #Initialize the getSdf class
        sdfFile = GetSDF()

        #Set up the spherical coordinates
        sdfFile.setOffset(self.args.boundingbox)
        sdfFile.addGroundPlane(self.args.boundingbox)
        sdfFile.addSphericalCoords(self.args.boundingbox)
        
        #Add Required models
        for model in modelPoseMap.keys():
            points = modelPoseMap[model]['points']
            sdfFile.addModel(modelPoseMap[model]['mainModel'],
                            model,
                            [points[0, 0], points[1, 0], points[2, 0]])
            

        for building in buildingLocationMap.keys():
            
            sdfFile.addBuilding(buildingLocationMap[building]['mean'],
                                buildingLocationMap[building]['points'],
                                building,
                                buildingLocationMap[building]['color'])

        #Include the roads in the map in sdf file
        for road in roadPointWidthMap.keys():
            sdfFile.addRoad(road)
            sdfFile.setRoadWidth(roadPointWidthMap[road]['width'], road)
            points = roadPointWidthMap[road]['points']
            for point in range(len(points[0, :])):
                sdfFile.addRoadPoint([points[0, point],
                                    points[1, point],
                                    points[2, point]],
                                    road)

        #Output sdf File
        sdfFile.writeToFile(self.args.outFile)

if __name__ == "__main__":
    parser = OsmParser()
    parser.parse_osm_()