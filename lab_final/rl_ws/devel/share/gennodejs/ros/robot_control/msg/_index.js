
"use strict";

let rigid = require('./rigid.js');
let tactile = require('./tactile.js');
let fsrInput = require('./fsrInput.js');
let accelerometr = require('./accelerometr.js');
let state = require('./state.js');
let newtactile = require('./newtactile.js');
let contact = require('./contact.js');
let coord = require('./coord.js');

module.exports = {
  rigid: rigid,
  tactile: tactile,
  fsrInput: fsrInput,
  accelerometr: accelerometr,
  state: state,
  newtactile: newtactile,
  contact: contact,
  coord: coord,
};
