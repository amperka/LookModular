// Сервопривод, управляющий расстоянием между
// датчиком линии и вращающейся картинкой
var envelopeServo = require('@amperka/servo').connect(P13);
// MIDI-вход на ножке P0 iskraJS
var midi = require('Midi').setup(PrimarySerial, 31250);
// PID-регулятор для управления частотой вращения двигателя
var pid = require('@amperka/pid').create({kp: 0.2});
// Класс для создания плавных переходов
var Animation = require('@amperka/animation');
// Функция для перевода MIDI-сообщений в реальные величины - вольты, градусы...
var midiMapping = function(midiValue, outMin, outMax) {
  var midiMaxValue = 127;
  return outMin + midiValue * (outMax - outMin) / midiMaxValue;
};
// Объект для отслеживания состояний клавиш MIDI-клавиатуры
var NoteQueue = function() {
  this._pressedKeys = [];
};
NoteQueue.prototype.push = function(note) {
  this._pressedKeys.push(note);
};
NoteQueue.prototype.delete = function(note) {
  var notePosition;
  while ((notePosition = this._pressedKeys.indexOf(note)) !== -1) {
    this._pressedKeys.splice(notePosition, 1);
  }
};
NoteQueue.prototype.lastNote = function() {
  var keysCount = this._pressedKeys.length;
  return (keysCount) ? this._pressedKeys[keysCount - 1] : null;
};

// Объект для управления частотой двигателя
var CdDrive = function(opts) {
  // Двигатель управляется напряжением. Считываем опорное напряжение,
  // чтобы иметь эталон напряжения
  var vRef = 0, avgCount = 8;
  for (var c = 0; c < avgCount; c++) {
    vRef += E.getAnalogVRef();
  }
  vRef /= avgCount;
  this._oscPin = opts.oscPin;
  this._speedPin = opts.speedPin;
  // Приводим заданные границы напряжения управления двигателем
  // в относительные величины
  this._minSpeed = opts.minSpeedV / vRef;
  this._maxSpeed = opts.maxSpeedV / vRef;
  this._pid = opts.pid;
  // Рассчитываем управляющее воздействие PID-регулятора каждые 5 мс
  this._pidUpdatePeriod = opts.updatePeriod || 0.005;
  this._coilSwitchPeriod = 0;
  // Обмотки двигателя переключаются 18 раз за оборот
  var coilSwitchesPerRevolution = 18;
  // При каждом переключении драйвер двигателя генерирует импульс.
  // Этот импульс вызовет у 2 прерывания - по переднему и заднему фронту
  var irqPerSwitch = 2;
  this._revolutionPerPulses = 1 / (coilSwitchesPerRevolution * irqPerSwitch);
  this._pid.setup({
    target: 0,
    outputMin: this._minSpeed,
    outputMax: this._maxSpeed
  });
  // Пока что в Espruino нет простого способа быстро посчитать
  // количество импульсов не дёргая при этом интерпритатор JavaScript.
  // Это мы записали себе в TODO, а пока воспользуемся ассемблерной вставкой.
  // Эта функция считает, сколько раз её вызвали, и возвращает это значение.
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
  // В прерывании по изменению уровня будем вызывать ассемблерную фнукцию "+1"
  // опция irq: true говорит о том, что функция будет вызвана напрямую из
  // обработчика прерываний, без дёргания интерпретатора.
  // Подробнее :
  // http://www.espruino.com/Reference#l__global_setWatch
  // http://www.espruino.com/Assembler#setwatch
  setWatch(this._increment, self._oscPin, {irq: true});

  // Запускаем PID-регулятор. Считаем текущую частоту.
  // Пытаемся изменить напряжение на управляющем пине так,
  // чтобы частота совпадала с уставкой
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

// Задаём новую уставку по частоте
CdDrive.prototype.frequency = function(freq) {
  this._pid.setup({target: freq});
};

// Объект - огибающая уровня громкости
// Управляет сервоприводом для включения и выключения звука при помощи
// приближения и удаления датчика линии от вращающейся картинки.
// Позволяет медленно менять громкость звука, формируя ADSR-огибающую
var AmpEnvelope = function(opts) {
  this._servo = opts.servo;
  // Угол сервопривода, при котором громкость минимальна
  this._minAmp = opts.minAmpDeg;
  // Угол сервопривода, при котором громкость максимальна
  this._maxAmp = opts.maxAmpDeg;
  // Шаг изменения значения ADSR-огибающей - 50 мс
  this._envelopeUpdateTime = opts.envelopeUpdateTime || 0.025;
  this._attack = opts.attack || 0.01;
  this._hold = opts.hold || 0.1;
  this._decay = opts.decay || 0.1;
  var sustain = opts.sustain || 0.1;
  this._sustain = this._minAmp + (this._maxAmp - this._minAmp) * sustain;
  this._release = opts.release || 0.05;
  this._attackDecayAnim = undefined;
  this._releaseAnim = undefined;
  this._minPeriodLength = 0;
  this._maxPeriodLength = 5;
  this._currentAmp = this._minAmp;
  this._update = function(amp) {
    this._currentAmp = amp;
    this._servo.write(this._currentAmp);
  };
  this._controlToLength = function(control){
    return midiMapping(
      control,
      this._minPeriodLength,
      this._maxPeriodLength
    );
  };
};
AmpEnvelope.prototype.stop = function() {
  if (this._attackDecayAnim) {
    this._attackDecayAnim.stop();
    this._attackDecayAnim = undefined;
  }
  if (this._releaseAnim) {
    this._releaseAnim.stop();
    this._attackDecayAnim = undefined;
  }
  this._servo.write(this._currentAmp);
};
AmpEnvelope.prototype.noteOn = function(velocity) {
  this.stop();
  velocity = midiMapping(velocity, this._minAmp, this._maxAmp);
  this._attackDecayAnim = Animation.create({
    from: this._currentAmp,
    to: velocity,
    duration: this._attack,
    updateInterval: this._envelopeUpdateTime
  }).queue({
    to: velocity,
    duration: this._hold
  }).queue({
    to: this._sustain,
    duration: this._decay
  });
  var self = this;
  this._attackDecayAnim.on('update', function(val) {
    self._update(val);
  });
  this._attackDecayAnim.play();
};
AmpEnvelope.prototype.noteOff = function() {
  this.stop();
  this._releaseAnim = Animation.create({
    from: this._currentAmp,
    to: this._minAmp,
    duration: this._release,
    updateInterval: this._envelopeUpdateTime
  });
  var self = this;
  this._releaseAnim.on('update', function(val) {
    self._update(val);
  });
  this._releaseAnim.play();
};
AmpEnvelope.prototype.attack = function(control) {
  this._attack = this._controlToLength(control);
};
AmpEnvelope.prototype.decay = function(control) {
  this._decay = this._controlToLength(control);
};
AmpEnvelope.prototype.release = function(control) {
  this._release = this._controlToLength(control);
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

var keyboard = new NoteQueue();

var midiChannel = 0;
var decayCC = 1;

ampEnvelope.noteOff();

midi.on('noteOn', function(i) {
  if (i.chan === midiChannel) {
    ampEnvelope.noteOn(i.velocity);
    if (i.velocity !== 0) {
      keyboard.push(i.note);
      cdDrive.frequency(midiNotesFrequency[keyboard.lastNote()]);
    }
  }
});

midi.on('ctrlChange', function(i) {
  if ((i.chan === midiChannel) && (i.ctrl === decayCC)) {
    ampEnvelope.decay(i.value);
  }
});

midi.on('noteOff', function(i) {
  if (i.chan === midiChannel) {
    keyboard.delete(i.note);
    var lastNote = keyboard.lastNote();
    if (lastNote) {
      cdDrive.frequency(midiNotesFrequency[lastNote]);
    } else {
      ampEnvelope.noteOff();
    }
  }
});
