// Сервопривод, управляющий расстоянием между
// датчиком линии и вращающейся картинкой
var envelopeServo = require('@amperka/servo').connect(P13);
// MIDI-вход на ножке P0 iskraJS
var midi = require('Midi').setup(PrimarySerial, 31250);
// PID-регулятор для управления частотой вращения двигателя
var pid = require('@amperka/pid').create({kp: 0.2});
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
  // Этот импульс вызовет у нас 2 прерывания - по переднему и заднему фронту
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
  // Шаг изменения значения ADSR-огибающей - 25 мс
  // Из всего ADSR я буду использовать только hold и decay
  this._decayUpdateTime = opts.decayUpdateTime || 25;
  // Время, в течении которого звук ноты максимальный.
  // За это время серва должна успеть повернуться до громкости ноты.
  this._holdTime = opts.holdTime || 100;
  // Коэффициенты для расчёта следующего уровня громкости
  // во время decay-периода.
  this._decayMinK = 0.9;
  this._decayMaxK = 1;
  // При старте затухания не будет.
  // Потом можем поменять через control change MIDI-клавиатуры
  this._currentDecayK = this._decayMaxK;
  // Sustain level круто было бы задавать с MIDI-клавиатуры,
  // но моя такого не позволяет. Хардкод:
  this._sustainLevel = this._minAmp + (this._maxAmp - this._minAmp) * 0.1;
  this._velocity = null;
  this._decayIntervalID = null;
  this._holdTimeoutID = null;
};
AmpEnvelope.prototype.stop = function() {
  if (this._holdTimeoutID) {
    clearTimeout(this._holdTimeoutID);
    this._holdTimeoutID = null;
  }
  if (this._decayIntervalID) {
    clearInterval(this._decayIntervalID);
    this._decayIntervalID = null;
  }
};
AmpEnvelope.prototype.run = function(velocity) {
  this._velocity = midiMapping(velocity, this._minAmp, this._maxAmp);
  this.stop();
  this._servo.write(this._velocity);
  var self = this;
  this._holdTimeoutID = setTimeout(function(){
    self._decayIntervalID = setInterval(function(){
      var newVolumePart = self._sustainLevel * (1 - self._currentDecayK);
      var oldVolumePart = self._velocity * self._currentDecayK;
      self._velocity = newVolumePart + oldVolumePart;
      self._servo.write(self._velocity);
    }, self._decayUpdateTime);
    self._holdTimeoutID = null;
  }, this._holdTime);
};
AmpEnvelope.prototype.decay = function(control) {
  this._currentDecayK = midiMapping(control, this._decayMinK, this._decayMaxK);
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

ampEnvelope.run(0);

midi.on('noteOn', function(i) {
  if (i.chan === midiChannel) {
    ampEnvelope.run(i.velocity);
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
      ampEnvelope.run(0);
    }
  }
});
