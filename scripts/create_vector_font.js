#!/usr/bin/node
/*
 This file is part of Espruino, a JavaScript interpreter for Microcontrollers

 Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>

 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.

 ----------------------------------------------------------------------------------------
  Bitmap font creator for Graphics custom fonts

  This is pretty rough-and-ready, and in order to easily get bitmap data out of an image
  it requires the image to already be in RAW format.

  For this, use ImageMagick as follows: `convert charset_8x12.png -depth 8 gray:charset_8x12.raw`
 ----------------------------------------------------------------------------------------
*/

// npm install opentype.js

var FILENAME = "Roboto-Black.ttf";
var OUTFILE = "../libs/graphics/vector_font.h";

// =============================================================================

function dist(a,b) {
  var dx = a.x-b.x;
  var dy = a.y-b.y;
  return Math.sqrt(dx*dx+dy*dy);
}

function isInside(point, poly) {
  // ray-casting algorithm based on
  // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
  var x = point.x, y = point.y;
  var inside = false;
  for (var i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    var xi = poly[i].x, yi = poly[i].y;
    var xj = poly[j].x, yj = poly[j].y;

    var intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
};

function polygonArea(poly) {
  //https://stackoverflow.com/questions/14505565/detect-if-a-set-of-points-in-an-array-that-are-the-vertices-of-a-complex-polygon
  var area = 0;
  for (var i = 0; i < poly.length; i++) {
    j = (i + 1) % poly.length;
    area += poly[i].x * poly[j].y;
    area -= poly[j].x * poly[i].y;
  }
  return area / 2;
}

// Move a small distance away from path[idxa] towards path[idxb]
function interpPt(path, idxa, idxb) {
  var amt = 0;
  // wrap index
  if (idxb<0) idxb+=path.length;
  if (idxb>=path.length) idxb-=path.length;
  // get 2 pts
  var a = path[idxa];
  var b = path[idxb];
  var dx = b.x - a.x;
  var dy = b.y - a.y;
  var d = Math.sqrt(dx*dx + dy*dy);
  if (amt > d) return []; // return nothing - will just end up using the last point
  return [{
    x : a.x + (dx*amt/d),
    y : a.y + (dy*amt/d)
  }];
}

function unpackPoly(poly) {
  // ensure all polys are the right way around
  for (var p=0;p<poly.length;p++) {
    if (polygonArea(poly[p])>0)
      poly[p].reverse();
  }
  var finalPolys = [poly[0]];
  for (var p=1;p<poly.length;p++) {
    var path = poly[p];

    var outerPolyIndex = undefined;
    for (var i=0;i<finalPolys.length;i++) {
      if (isInside(path[0], finalPolys[i])) {
        outerPolyIndex = i;
        break;
      } else if (isInside(finalPolys[i], path)) {
        // polys in wrong order - old one is inside new one
        var t = path;
        path = finalPolys[i];
        finalPolys[i] = t;
        outerPolyIndex = i;
        break;
      }
    }

    if (outerPolyIndex!==undefined) {
      path.reverse(); // reverse poly
      var outerPoly = finalPolys[outerPolyIndex];
      var minDist = 10000000000;
      var minOuter,minPath;
      for (var a=0;a<outerPoly.length;a++)
        for (var b=0;b<path.length;b++) {
          var l = dist(outerPoly[a],path[b]);
          if (l<minDist) {
            minDist = l;
            minOuter = a;
            minPath = b;
          }
        }
      // splice the inner poly into the outer poly
      // but we have to recess the two joins a little
      // otherwise Eagle reports Invalid poly when filling
      // the top layer
      finalPolys[outerPolyIndex] =
        outerPoly.slice(0, minOuter).concat(
          interpPt(outerPoly,minOuter,minOuter-1),
          interpPt(path,minPath,minPath+1),
          path.slice(minPath+1),
          path.slice(0,minPath),
          interpPt(path,minPath,minPath-1),
          interpPt(outerPoly,minOuter,minOuter+1),
          outerPoly.slice(minOuter+1)
        );
    } else {
      // not inside, just add this poly
      finalPolys.push(path);
    }
  }
  return finalPolys;
}

function quadraticBezier(pp,x0,y0,x1,y1,x2,y2){
  var x,y,t,t2;
  var sn = 3;

  //pp.push({x:x0,y:y0});
  var s = Math.min(sn/Math.abs(x0-x2),sn/Math.abs(y0-y2));
  for( t = s; t <= 1; t += s) {
    t2 = t*t;
    tp2 = (1 - t) * (1 - t);
    x = ( x0 * tp2 +
        x1 * 2 * (1 - t) * t +
        t2 * x2 + 0.5) | 0;
    y = ( y0 * tp2 +
        y1 * 2 * (1 - t) * t +
        t2 * y2 + 0.5 ) | 0 ;
    pp.push({x:x,y:y});
  }
  pp.push({x:x2,y:y2});
}

function cleanUp(poly) {
  var last = poly[poly.length-1];
  for (var i=0;i<poly.length;i++) {
    if (poly.length==1) return;
    if (dist(last,poly[i])<0.5) {
      poly.splice(i,1); // delete one
      i--; // step back
    } else last = poly[i];
  }
}
// ==============================================================

var opentype = require('opentype.js');


opentype.load('Roboto-Black.ttf', function(err, font) {
  if (err) {
    console.log('Font could not be loaded: ' + err);
    process.exit(1);
  }
  writeFile(font);
});

// Rescale for output to array
function rescalePolys(polys, c) {
  for (var p=0;p<polys.length;p++) {
    for (var i=0;i<polys[p].length;i++) {
      var x = Math.round(polys[p][i].x);
      var y = Math.round(polys[p][i].y + 96);
      if (x<0) {
        x=0;
        console.log("X Coordinate for ch "+c+" out of range");
      }
      if (y<0) {
        y=0;
        console.log("Y Coordinate for ch "+c+" out of range");
      }
      if (x>127) {
        x=127;
        console.log("X Coordinate for ch "+c+" out of range");
      }
      if (y>127) {
        y=127;
        console.log("Y Coordinate for ch "+c+" out of range");
      }
      polys[p][i].x = x;
      polys[p][i].y = y;
    }
  }
}

function pathToPolys(path) {
  var fullpaths = [];
  var pts = [];
  console.log(path.commands);
  path.commands.forEach(cmd => {
    switch (cmd.type) {
      case "M":
      case "L":
        pts.push({x:cmd.x, y:cmd.y});
        break;
      case "Z":
        pts.push({x:pts[0].x, y:pts[0].y});
        fullpaths.push(pts);
        pts=[];
        break;
      case "Q":
        // very hacky quadratic, just add midpoint
        pts.push({
          x:(pts[pts.length-1].x+cmd.x+cmd.x1*2)/4,
          y:(pts[pts.length-1].y+cmd.y+cmd.y1*2)/4
        });
        pts.push({x:cmd.x, y:cmd.y});
        // we should be able to just do this - not working right now though
        //quadraticBezier(pts, pts[pts.length-1].x, pts[pts.length-1].y, cmd.x1, cmd.y1, cmd.x, cmd.y);
        break;
      default: console.log("Unhandled "+cmd.type);
    }
  });
  if (pts.length) {
    fullpaths.push(pts);
    pts=[];
  }
  console.log(fullpaths);
  rescalePolys(fullpaths);
  fullpaths.forEach(cleanUp);
  if (fullpaths.length)
    fullpaths = unpackPoly(fullpaths);
  console.log(fullpaths);
  fullpaths.forEach(cleanUp);
  /*for (var p=0;p<fullpaths.length;p++) {
    for (var i=0;i<fullpaths[p].length;i++)
      fullpaths[p][i] = 128+Math.round(fullpaths[p][i]);
  }*/
   return fullpaths;
}

function writeFile(font) {
  var fontCoords = "";
  var fontOffsets = "";
  var VECTOR_FONT_MAX_POLY_SIZE = 0;
  for (var c=32;c<128;c++) {
    console.log("Character code "+c);
    var glyph = font.charToGlyph(String.fromCharCode(c));
    var fontSize = 128;
    var path = glyph.getPath(0, 0, fontSize);
    var width = glyph.advanceWidth / (glyph.path.unitsPerEm || 1000) * fontSize;
    var polys = pathToPolys(path);
    fontCoords += "// Character code "+c+"\n";
    var n = 0;
    for (var p=0;p<polys.length;p++) {
      if (polys[p].length > VECTOR_FONT_MAX_POLY_SIZE)
        VECTOR_FONT_MAX_POLY_SIZE = polys[p].length;
      for (var i=0;i<polys[p].length;i++) {
        var last = i==polys[p].length-1;
        var x = Math.round(polys[p][i].x);
        var y = Math.round(polys[p][i].y);
        fontCoords += (x+","+y)+(last?"|VECTOR_FONT_POLY_SEPARATOR":"")+",\n";
        n++;
      }
    }
    fontOffsets += "{"+Math.round(width)+", "+(n*2)+"}, // char "+c+"\n";
  }

  var f = `
/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * Machine geneated vector font header (by create_vector_font.js)
 * ----------------------------------------------------------------------------
 */
// Created for ${FILENAME}
#define VECTOR_FONT_IS_EDGE 128 // applied to X coord if this point and the next are on an edge
#define VECTOR_FONT_POLY_SEPARATOR 128 // applied to Y coord if the end of a poly
#define VECTOR_FONT_POLY_SIZE 96 // the actual size of the font
static const unsigned char vectorFontPolys[] IN_FLASH_MEMORY = {
${fontCoords}
};
const unsigned char VECTOR_FONT_MAX_POLY_SIZE = ${VECTOR_FONT_MAX_POLY_SIZE};
typedef struct VectorFontChar {
  unsigned char width;
  unsigned char vertCount; // 2x (for x and y)
} VectorFontChar;
static const int vectorFontOffset = 32;
static const int vectorFontCount = 95;
static const VectorFontChar vectorFonts[] IN_FLASH_MEMORY = {
${fontOffsets}
};
`;
  console.log(f);
  require("fs").writeFileSync(OUTFILE, f);
}
