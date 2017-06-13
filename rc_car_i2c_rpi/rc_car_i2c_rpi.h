typedef enum {
  BAD_,               //  0
  OK_,                //  1
  CHECK_SONAR,        //  2 return 0-4 0 = ok, 1-3 left, center, right
  CHECK_SONAR_LEFT,   //  3 return if left sonar is seeing something
  CHECK_SONAR_CENTER, //  4 return if center sonar is seeing something
  CHECK_SONAR_RIGHT,  //  5 return if right sonar is seeing something
  GO_RIGHT,           //  6 try to go right; return 0 if not ok 1 if ok
  GO_CENTER,          //  7 try to go to the center; return 0 if not ok 1 if ok
  GO_LEFT,            //  8 try to go left; return 0 if not ok 1 if ok
  GET_DIR,            //  9 return where the servo is, left, center, or right
  
  SONAR_LEFT,         // 10
  SONAR_CENTER,       // 11
  SONAR_RIGHT,        // 12

  NOCHANGE_,          // 13
  RIGHT_,             // 14
  LEFT_,              // 15
  CENTER_,            // 16
  FAR_RIGHT_,         // 17
  FAR_LEFT_           // 18
} my_enum;

