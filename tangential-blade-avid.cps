/**
  Tangential knife post processer for Fusion 360.
  
  Based on work by jejmule. https://github.com/jejmule/PostProcessor
  
  For use with TwinCAT (Q1=) and knife oriented towards +Y axis.
*/

description = "Tangential Rotary Blade (TwinCAT - Y Axis Knife)";
vendor = "KBuchka";
vendorUrl = "kbuchka@gmail.com";
certificationLevel = 2;

longDescription = "Tangential Rotary Blade support based on Jejmule's post. Modified for TwinCAT (Q1=) with knife aligned to +Y axis.";

extension = "nc";
setCodePage("ascii");

capabilities = CAPABILITY_MILLING | CAPABILITY_MACHINE_SIMULATION;
allowHelicalMoves = false;
tolerance = spatial(0.3, MM);

minimumChordLength = spatial(0.25, MM);
minimumCircularRadius = spatial(0, MM);
maximumCircularRadius = spatial(1000, MM);
minimumCircularSweep = toRad(0.01);
maximumCircularSweep = toRad(180);
allowedCircularPlanes = 1 << PLANE_XY;

// user-defined properties
properties = {
  liftAtCorner: {
      title: "Lift Angle", 
      description: "Maximum angle at which the blade is turned in the material. If the angle is larger the blade is lifted and rotated.", 
      type: "angle", 
      value: 0.5
  },
  minLinearRadius: {
      title: "Minimum Radius", 
      description: "Absolute minimum radius allowable. Radii smaller than this will be clipped entirely with a linear move.", 
      type: "number", 
      value:5
  },
  minArcRadius: {
      title: "Minimum Arc Radius", 
      description: "Radii smaller than this will be approximated with discrete linear moves.", 
      type: "number", 
      value:10
  },
  usePostTolerance: {
      title: "Use Global Tolerance", 
      description: "Override operation-specific tolerances for linearizations.", 
      type: "boolean", 
      value: true
  },
  forceRapids: {
      title: "Force Rapid Moves", 
      description: "Un-nerf the free version of Fusion by trying to force G0 rapid moves instead of G1 rapid moves. USE AT YOUR OWN RISK.", 
      type: "boolean", 
      value: true
  },
  useCalcAngularFeed: {
      title: "Use calculated angular feed",
      description: "Enabling this will compute an angular feedrate for G02/03 moves that is equivalent to the specified linear feedrate.",
      type: "boolean",
      value: true
  },
  reduceRotations: {
      title: "Reduce Rotations",
      description: "Enable this to minimize superfluous C-axis rotations.",
      type: "boolean",
      value: true
  },
  printDebug: {
      title: "Print Debug Strings",
      description: "Enable this to print additional debug information.",
      type: "boolean",
      value: false
  }
};

var WARNING_WORK_OFFSET = 0;
var WARNING_COOLANT = 1;

var gFormat = createFormat({prefix:"G", decimals:0, width:2, zeropad:true});
var mFormat = createFormat({prefix:"M", decimals:0});

var xyzFormat = createFormat({decimals:(unit == MM ? 3 : 4)});
var abcFormat = createFormat({decimals:3, forceDecimal:true});
var feedFormat = createFormat({decimals:(unit == MM ? 2 : 3)});

var xOutput = createVariable({prefix:"X"}, xyzFormat);
var yOutput = createVariable({prefix:"Y"}, xyzFormat);
var zOutput = createVariable({prefix:"Z"}, xyzFormat);

// Użycie nazewnictwa Q1= dla TwinCAT
var cOutput = createVariable({prefix:"Q1="}, abcFormat);

var iOutput = createReferenceVariable({prefix:"I"}, xyzFormat);
var jOutput = createReferenceVariable({prefix:"J"}, xyzFormat);
var feedOutput = createVariable({prefix:"F"}, feedFormat);

var gMotionModal = createModal({}, gFormat); // modal group 1 // G0-G3, ...
var gAbsIncModal = createModal({}, gFormat); // modal group 3 // G90-91

var sequenceNumber = 0;

// KNIFE ALIGNMENT OFFSET: -90 stopni (-PI/2 rad) ponieważ nóż patrzy na +Y
var KNIFE_OFFSET_RAD = -Math.PI / 2.0;

// Pozycja początkowa osi Q (pozycja bazowa z uwzględnieniem offsetu noża)
var c_rad = KNIFE_OFFSET_RAD; 
var isRapid = false;

/**
 Update C position for Tangential Rotary Blade
 */
 function updateC(target_rad) {
  var delta_rad = (target_rad-c_rad);

  if (delta_rad % (2*Math.PI) == 0){
    return;
  }
  
  if (Math.abs(delta_rad) > toRad(getProperty("liftAtCorner"))) { 
    moveUp();
    gMotionModal.reset();
    writeBlock(gMotionModal.format(0), cOutput.format(toDeg(target_rad)));
    moveDown();
    c_rad = target_rad;
  }
  else { 
    writeBlock(gMotionModal.format(1), cOutput.format(toDeg(target_rad)));
    c_rad = target_rad;
  }
 }
 
/**
 Update C position with minimal rotations
 */
 function updateCminRotation(target_rad) {
  var delta_rad = (target_rad-c_rad);

  if (delta_rad % (2*Math.PI) == 0){
    return;
  }
  
  if (Math.abs(delta_rad) > toRad(getProperty("liftAtCorner"))) { 
    moveUp();

    var currentNormalized = Math.abs(((c_rad % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI));
    var targetNormalized = Math.abs(((target_rad % (2*Math.PI)) + (2*Math.PI)) % (2*Math.PI));
    
    var deltaNormalized = targetNormalized - currentNormalized;
    
    writeDebug("Curr: " + toDeg(c_rad) + " Targ: " + toDeg(target_rad));
    writeDebug("CurrNorm: " + toDeg(currentNormalized) + " TargNorm: " + toDeg(targetNormalized));
    
    if (Math.abs(delta_rad) > Math.PI) {
        if (Math.abs(deltaNormalized) > Math.PI) {
            gAbsIncModal.reset();
            if (currentNormalized < targetNormalized) {
                writeDebug("Zero cross, current nearer 0");
                writeBlock(gAbsIncModal.format(91), cOutput.format(toDeg(-(currentNormalized + (2*Math.PI - targetNormalized)))));
            } else {
                writeDebug("Zero cross, target nearer 0");
                writeBlock(gAbsIncModal.format(91), cOutput.format(toDeg((targetNormalized+(2*Math.PI-currentNormalized)))));
            }
        } else {
            writeDebug("Inside 180 in normalized coords");
            writeBlock(gAbsIncModal.format(91), cOutput.format(toDeg(deltaNormalized)));
        }
        writeBlock(gAbsIncModal.format(90));
        writeBlock(gAbsIncModal.format(92), cOutput.format(toDeg(target_rad)));
    } else {
        writeDebug("Inside 180 in absolute coords");
        gMotionModal.reset();
        writeBlock(gMotionModal.format(0), cOutput.format(toDeg(target_rad)));
    }
    
    moveDown();
    c_rad = target_rad;
  }
  else { 
    writeBlock(gMotionModal.format(1), cOutput.format(toDeg(target_rad)));
    c_rad = target_rad;
  }
 }

/**
 Move cutter up to retract height
 */
 function moveUp() {
   retractPos = getCurrentPosition();
   onRapid(retractPos.x,retractPos.y,getParameter("operation:retractHeight_value"));
 }

/**
 Move cutter down to work height
 */
function moveDown() {
  plungePos = getCurrentPosition();
  var z = zOutput.format(plungePos.z); 
  if (z) {
    gMotionModal.reset();
    var pFeed = (tool.plungeFeedrate > 0) ? tool.plungeFeedrate : 400;
    writeBlock(gMotionModal.format(1), z, feedOutput.format(pFeed));
  }
}

/**
 Writes the specified block.
*/
function writeBlock() {
  var blockStr = formatWords(arguments);

  blockStr = blockStr.replace("G00", "G01");

  if (blockStr.indexOf("Z") !== -1 && blockStr.indexOf("X") === -1 && blockStr.indexOf("Y") === -1) {
    blockStr = blockStr.replace(/F[0-9.]+/g, ""); 
    blockStr += " F400"; 
  } 
  else if (blockStr.indexOf("G01") !== -1 && blockStr.indexOf("F") === -1) {
    blockStr += " F1500"; 
  }

  blockStr = blockStr.replace(/\s+/g, ' ').trim();
  
  writeln("N" + sequenceNumber + " " + blockStr);
  sequenceNumber += 1;
}

function writeComment(text) {
  writeln("(" + text + ")");
}

function writeDebug(text) {
    if (getProperty("printDebug")) {
        writeComment(text);
    }
}

function onOpen() {
  if (programName) {
    writeComment(programName);
  }
  if (programComment) {
    writeComment(programComment);
  }
}

function onSection() {
  if (!isSameDirection(currentSection.workPlane.forward, new Vector(0, 0, 1))) {
    error(localize("Tool orientation is not supported."));
    return;
  }
  setRotation(currentSection.workPlane);

  if (currentSection.workOffset != 0) {
    warningOnce(localize("Work offset is not supported."), WARNING_WORK_OFFSET);
  }
  if (tool.coolant != COOLANT_OFF) {
    warningOnce(localize("Coolant not supported."), WARNING_COOLANT);
  }

  c_rad = KNIFE_OFFSET_RAD;
  feedOutput.reset();
}

function onRapid(_x, _y, _z) {
  isRapid = true;
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z);
  
  if (x || y || z) {
    writeBlock(gMotionModal.format(0), x, y, z);
    feedOutput.reset();
  }
}

function onLinear(_x, _y, _z, feed) {
  var start = getCurrentPosition();
  var target = new Vector(_x,_y,_z);
  var direction = Vector.diff(target,start);
  
  // ZMIANA: Dodano KNIFE_OFFSET_RAD do wyliczania kąta
  var orientation_rad = direction.getXYAngle() + KNIFE_OFFSET_RAD;
  
  if (!(start.x == _x && start.y == _y)) {
      if (getProperty("reduceRotations")) {
          updateCminRotation(orientation_rad);
      } else {
          updateC(orientation_rad);
      }
  }
  
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z); // Pobiera dokładną wysokość z Fusion 360

  if (x || y || z) {
    writeBlock(gMotionModal.format(1), x, y, z, feedOutput.format(feed));
  }
}

function onCircular(clockwise, cx, cy, cz, x, y, z, feed) {
  var radius = getCircularRadius();

  if (radius <= getProperty("minLinearRadius")) {
    onLinear(x, y, x, feed);
    return;
  }
  
  if (radius <= getProperty("minArcRadius")) {
    var t = tolerance;
    if (hasParameter("operation:tolerance") && !getProperty("usePostTolerance")) {
      t = getParameter("operation:tolerance");
    }
    linearize(t);
    return;
  }

  switch (getCircularPlane()) {
  case PLANE_XY:
    var arcLength = getCircularArcLength();
    var arcAngle = getCircularSweep();
    
    var start = getCurrentPosition();
    var OD = start;
    var OC = getCircularCenter();
    
    var Z = new Vector(0,0,clockwise ? 1 : -1);
    var CD = Vector.diff(OD,OC);
    var tangent = Vector.cross(CD,Z);
    
    // ZMIANA: Dodano KNIFE_OFFSET_RAD do kierunku startowego na łuku
    var start_dir = tangent.getXYAngle() + KNIFE_OFFSET_RAD;

    if (getProperty("reduceRotations")) {
        updateCminRotation(start_dir);
    } else {
        updateC(start_dir);
    }

    if(clockwise){
      c_rad -= arcAngle;
    }
    else {
      c_rad += arcAngle;
    }
    
    var outputFeed = feed;
    if (getProperty("useCalcAngularFeed")) {
        outputFeed = calcAngularFeed(arcLength, arcAngle, feed);
    }
    
    feedOutput.reset(); 
writeBlock(gMotionModal.format(clockwise ? 2 : 3), xOutput.format(x), yOutput.format(y), cOutput.format(toDeg(c_rad)), iOutput.format(cx - start.x, 0), jOutput.format(cy - start.y, 0), feedOutput.format(outputFeed));
    break;
  default:
    var t = tolerance;
    if (hasParameter("operation:tolerance")) {
      t = getParameter("operation:tolerance");
    }
    linearize(t);
  }
}

function calcAngularFeed(arcLength, arcAngle, linearFeed) {
    travelTime = arcLength/linearFeed;
    return toDeg(arcAngle)/travelTime;
}

function onSectionEnd() {
  moveUp();
}

function onClose() {

}