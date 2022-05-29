
uint16_t valve_open[4] = {
    1300,
    1300,
    1300,
    1300
};

uint16_t valve_closed[4] = {
    1150,
    1300,
    1300,
    1300
};

const struct lut {
  uint16_t speed;
  uint16_t angle;
} mainLut[4][128] = {
    {},
    {},
    {},
    {}
};
