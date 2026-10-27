description = "Tangential Rotary Blade";
vendor = "Vergano";
vendorUrl = "damian.jan@vergano.ch";
certificationLevel = 2;
version = "1.0";

longDescription = "Tangential Rotary Blade support. Modified for TwinCAT (Q1=) with knife aligned to +Z axis. NOTE: All parameters are controlled from this post processor's menu. Options in the 'Built-in' tab should be ignored.";

extension = "nc";
setCodePage("ascii");

capabilities = CAPABILITY_MILLING | CAPABILITY_MACHINE_SIMULATION;
allowHelicalMoves = false;
tolerance = spatial(0.05, MM);

minimumChordLength = spatial(0.25, MM);
minimumCircularRadius = spatial(0, MM);
maximumCircularRadius = spatial(1000, MM);
minimumCircularSweep = toRad(0.01);
maximumCircularSweep = toRad(180);
allowedCircularPlanes = 1 << PLANE_XY;

properties = {
  cuttingFeedrate: {
      title: "Cutting feedrate",
      description: "Main working feedrate for XY axis movements along the cut contour.",
      type: "number",
      value: 1000,
      group: "Feeds and Speeds"
  },
  plungeFeedrate: {
      title: "Plunge feedrate",
      description: "Feedrate for lowering and plunging the blade in the Z axis.",
      type: "number",
      value: 400,
      group: "Feeds and Speeds"
  },
  rotationFeedrate: {
      title: "Rotation feedrate",
      description: "Rotation speed of the Q1 axis alone in the air above a corner.",
      type: "number",
      value: 666,
      group: "Feeds and Speeds"
  },
  highFeedrate: {
      title: "Rapid feedrate",
      description: "Feedrate for rapid moves in the air between contours.",
      type: "number",
      value: 3000,
      group: "Feeds and Speeds"
  },
  liftAtCorner: {
      title: "Corner lift angle",
      description: "Angular threshold above which the blade lifts and rotates.",
      type: "angle",
      value: 0.5,
      group: "Geometry and Knife Operation"
  },
  minArcRadius: {
      title: "Minimum arc radius",
      description: "Threshold for linearizing arcs into line segments.",
      type: "number",
      value: 10,
      group: "Geometry and Knife Operation"
  },
  minLinearRadius: {
      title: "Absolute minimum radius",
      description: "Radius replaced by a single linear move.",
      type: "number",
      value: 5,
      group: "Geometry and Knife Operation"
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

var cOutput = createVariable({prefix:"Q1="}, abcFormat);

var iOutput = createReferenceVariable({prefix:"I"}, xyzFormat);
var jOutput = createReferenceVariable({prefix:"J"}, xyzFormat);
var feedOutput = createVariable({prefix:"F"}, feedFormat);

var gMotionModal = createModal({}, gFormat);
var gAbsIncModal = createModal({}, gFormat);

var sequenceNumber = 0;

var KNIFE_OFFSET_RAD = -Math.PI / 2.0;

var c_rad = KNIFE_OFFSET_RAD; 
var isRapid = false;

var pendingPlungeZ = undefined;

function updateC(target_rad) {
  var delta_rad = (target_rad - c_rad);

  var normalized_delta = delta_rad % (2 * Math.PI);
  if (normalized_delta > Math.PI) normalized_delta -= 2 * Math.PI;
  if (normalized_delta < -Math.PI) normalized_delta += 2 * Math.PI;

  if (Math.abs(normalized_delta) < 1e-3) {
    c_rad += normalized_delta; 
    return;
  }
  
  var rotFeed = getProperty("rotationFeedrate");
  var isPhysicallyDown = (pendingPlungeZ === undefined);

  if (Math.abs(normalized_delta) > toRad(getProperty("liftAtCorner"))) { 
    if (isPhysicallyDown) moveUp();
    gMotionModal.reset();
    c_rad += normalized_delta;
    writeBlock(gMotionModal.format(1), cOutput.format(toDeg(c_rad)), feedOutput.format(rotFeed));
    if (isPhysicallyDown) moveDown();
  }
  else { 
    c_rad += normalized_delta;
    var cStr = cOutput.format(toDeg(c_rad));
    if (cStr) {
        writeBlock(gMotionModal.format(1), cStr, feedOutput.format(rotFeed));
    }
  }
}

function moveUp() {
   var retractZ = hasParameter("operation:retractHeight_value") ? getParameter("operation:retractHeight_value") : 15;
   retractPos = getCurrentPosition();
   onRapid(retractPos.x, retractPos.y, retractZ);
}

function moveDown() {
  plungePos = getCurrentPosition();
  var z = zOutput.format(plungePos.z); 
  if (z) {
    gMotionModal.reset();
    var pFeed = getProperty("plungeFeedrate");
    writeBlock(gMotionModal.format(1), z, feedOutput.format(pFeed));
    feedOutput.reset();
  }
}

function writeBlock() {
  var blockStr = formatWords(arguments);
  
  var isConvertedRapid = (blockStr.indexOf("G00") !== -1);
  
  blockStr = blockStr.replace("G00", "G01");

  if (blockStr.indexOf("Z") !== -1 && blockStr.indexOf("X") === -1 && blockStr.indexOf("Y") === -1) {
    var pFeed = getProperty("plungeFeedrate");
    blockStr = blockStr.replace(/F[0-9.]+/g, ""); 
    blockStr += " F" + feedFormat.format(pFeed);
  }
  else if (isConvertedRapid) {
    var cFeed = getProperty("highFeedrate");
    blockStr = blockStr.replace(/F[0-9.]+/g, ""); 
    blockStr += " F" + feedFormat.format(cFeed);
  }

  blockStr = blockStr.replace(/\s+/g, ' ').trim();
  writeln("N" + sequenceNumber + " " + blockStr);
  sequenceNumber += 1;
}

function writeComment(text) {
  writeln("(" + text + ")");
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
  pendingPlungeZ = undefined;
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
  
  if (start.x == _x && start.y == _y && _z < start.z) {
      pendingPlungeZ = _z;
      return; 
  }
  
  var direction = Vector.diff(target,start);
  var orientation_rad = direction.getXYAngle() + KNIFE_OFFSET_RAD;
  
  var retractZ = hasParameter("operation:retractHeight_value") ? getParameter("operation:retractHeight_value") : 15;
  var isTravel = (start.z >= retractZ - 0.01) && (_z >= retractZ - 0.01);

  if (!(start.x == _x && start.y == _y) && !isTravel) {
      updateC(orientation_rad);
  }
  
  if (pendingPlungeZ !== undefined) {
      var zStr = zOutput.format(pendingPlungeZ);
      if (zStr) {
          gMotionModal.reset();
          writeBlock(gMotionModal.format(1), zStr, feedOutput.format(getProperty("plungeFeedrate")));
          feedOutput.reset(); 
      }
      pendingPlungeZ = undefined;
  }
  
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z); 

  if (x || y || z) {
    var activeFeed = isTravel ? getProperty("highFeedrate") : getProperty("cuttingFeedrate");
    writeBlock(gMotionModal.format(1), x, y, z, feedOutput.format(activeFeed));
  }
}

function onCircular(clockwise, cx, cy, cz, x, y, z, feed) {
  var radius = getCircularRadius();

  if (radius <= getProperty("minLinearRadius")) {
    onLinear(x, y, z, 0); 
    return;
  }
  
  if (radius <= getProperty("minArcRadius")) {
    var t = tolerance;
    if (hasParameter("operation:tolerance")) {
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
    
    var start_dir = tangent.getXYAngle() + KNIFE_OFFSET_RAD;

    updateC(start_dir);

    if (pendingPlungeZ !== undefined) {
        var zStr = zOutput.format(pendingPlungeZ);
        if (zStr) {
            gMotionModal.reset();
            writeBlock(gMotionModal.format(1), zStr, feedOutput.format(getProperty("plungeFeedrate")));
            feedOutput.reset(); 
        }
        pendingPlungeZ = undefined;
    }

    if(clockwise){
      c_rad -= arcAngle;
    }
    else {
      c_rad += arcAngle;
    }
    
    var outputFeed = getProperty("cuttingFeedrate");
    
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

function onSectionEnd() {
  moveUp();
}

function onClose() {

}