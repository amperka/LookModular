var envelopeServo = require('@amperka/servo').connect(P13);
var midi = require('Midi').setup(PrimarySerial, 31250);
var pid = require('@amperka/pid').create({kp: 0.2});

var midiMapping = function(midiValue, outMin, outMax) {
  var midiMaxValue = 127;
  return outMin + midiValue * (outMax - outMin) / midiMaxValue;
};

var MidiKeyboardLogic = function() {
  this._pressedKeys = [];
};

MidiKeyboardLogic.prototype.noteOn = function(note) {
  this._pressedKeys.push(note);
};

MidiKeyboardLogic.prototype.noteOff = function(note) {
  var notePosition;
  while ((notePosition = this._pressedKeys.indexOf(note)) !== -1) {
    this._pressedKeys.splice(notePosition, 1);
  }
};

MidiKeyboardLogic.prototype.lastNote = function() {
  var keysCount = this._pressedKeys.length;
  return (keysCount) ? this._pressedKeys[keysCount - 1] : null;
};

var CdDrive = function(opts) {
  var vRef = 0, avgCount = 8;
  for (var c = 0; c < avgCount; c++) {
    vRef += E.getAnalogVRef();
  }
  vRef /= avgCount;
  this._pid = opts.pid;
  this._oscPin = opts.oscPin;
  this._speedPin = opts.speedPin;
  this._minSpeed = opts.minSpeedV / vRef;
  this._maxSpeed = opts.maxSpeedV / vRef;
  this._pidUpdatePeriod = opts.updatePeriod || 0.005;
  this._coilSwitchPeriod = 0;
  var coilSwitchesPerRevolution = 18;
  var irqPerSwitch = 2;
  this._revolutionPerPulses = 1 / (coilSwitchesPerRevolution * irqPerSwitch);
  this._pid.setup({
    target: 0,
    outputMin: this._minSpeed,
    outputMax: this._maxSpeed
  });

  this._increment = E.asm('int()',
    'adr    r1, data',
    'ldr    r0, [r1]',
    'add    r0, #1',
    'str    r0, [r1]',
    'bx lr',
    'nop',
    'data:',
    '.word    0x0'
  );

  this._lastPidUpdateTime = 0;
  this._interruptCount = 0;
  this._frequency = 0;

  var self = this;
  setWatch(this._increment, self._oscPin, {irq: true});

  this._pid.run(function() {
    var time = getTime() - self._lastPidUpdateTime;
    self._lastPidUpdateTime = getTime();
    var interruptCount = self._increment() - 1;
    var pulses = interruptCount - self._interruptCount;
    self._interruptCount = interruptCount;
    if (pulses > 0) {
      var revolutions = pulses * self._revolutionPerPulses;
      this._frequency = revolutions / time;
    }
    var output = self._pid.update(this._frequency);
    analogWrite(self._speedPin, output);
  }, this._pidUpdatePeriod);
};

CdDrive.prototype.frequency = function(freq) {
  this._pid.setup({target: freq});
};

var AmpEnvelope = function(opts) {
  this._servo = opts.servo;
  this._fadeMin = opts.fadeMin || 0.9;
  this._fadeMax = opts.fadeMax || 1;
  this._minAmp = opts.minAmpDeg;
  this._maxAmp = opts.maxAmpDeg;
  this._fadeUpdateTime = opts.fadeUpdateTime || 25;
  this._attackTime = opts.attackTime || 100;
  this._fading = this._fadeMax;
  this._decay = null;
  this._fadeIntervalID = null;
  this._decayTimeoutID = null;
};

AmpEnvelope.prototype.stop = function() {
  if (this._decayTimeoutID) {
    clearTimeout(this._decayTimeoutID);
    this._decayTimeoutID = null;
  }
  if (this._fadeIntervalID) {
    clearInterval(this._fadeIntervalID);
    this._fadeIntervalID = null;
  }
};

AmpEnvelope.prototype.run = function(velocity) {
  this._decay = midiMapping(velocity, this._minAmp, this._maxAmp);
  this.stop();
  this._servo.write(this._decay);
  var self = this;
  this._fadeIntervalID = setTimeout(function(){
    self._fadeIntervalID = setInterval(function(){
      var newDecayPart = self._minAmp * (1 - self._fading);
      var oldDecayPart = self._decay * self._fading;
      self._decay = newDecayPart + oldDecayPart;
      self._servo.write(self._decay);
    }, self._fadeUpdateTime);
    self._decayTimeoutID = null;
  }, this._attackTime);
};
AmpEnvelope.prototype.fade = function(control) {
  this._fading = midiMapping(control, this._fadeMin, this._fadeMax);
};

var midiNotesFrequency = new Float32Array(127);
for (var i = 0; i < midiNotesFrequency.length; i++) {
  var a1Freq = 440;
  var a1MidiNum = 69;
  midiNotesFrequency[i] = Math.pow(2, (i - a1MidiNum) / 12) * a1Freq;
}

var ampEnvelope = new AmpEnvelope({
  servo: envelopeServo,
  minAmpDeg: 45,
  maxAmpDeg: 79
});

var cdDrive = new CdDrive({
  pid: pid,
  oscPin: P2,
  speedPin: A4,
  minSpeedV: 1.4652,
  maxSpeedV: 1.6665
});

var keyboard = new MidiKeyboardLogic();

var midiChannel = 0;
var fadeCC = 1;

ampEnvelope.run(0);

midi.on('noteOn', function(i) {
  if (i.chan === midiChannel) {
    ampEnvelope.run(i.velocity);
    if (i.velocity !== 0) {
      keyboard.noteOn(i.note);
      cdDrive.frequency(midiNotesFrequency[keyboard.lastNote()]);
    }
  }
});

midi.on('ctrlChange', function(i) {
  if ((i.chan === midiChannel) && (i.ctrl === fadeCC)) {
    ampEnvelope.fade(i.value);
  }
});

midi.on('noteOff', function(i) {
  if (i.chan === midiChannel) {
    keyboard.noteOff(i.note);
    var lastNote = keyboard.lastNote();
    if (lastNote) {
      cdDrive.frequency(midiNotesFrequency[lastNote]);
    } else {
      ampEnvelope.run(0);
    }
  }
});
