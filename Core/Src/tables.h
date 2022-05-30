
const uint16_t valve_open[4] = {
    1250,
    1200,
    1310,
    1150
};

const uint16_t valve_closed[4] = {
    1150,
    1100,
    1220,
    1050
};

const uint16_t arm_limit[4] = {
    5,
    5,
    5,
    5
};

#define TABLE_SCALE 6

const struct lut {
  uint16_t speed;
  uint16_t angle;
} mainLut[4][128*TABLE_SCALE] = {
    {},
    {},
    {},
    {}
};
