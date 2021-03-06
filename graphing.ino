//****************************************************************************
// Based on https://github.com/alitai/GS3-Chimera
//***************************************************************************

#define GRAPH_X_0 3
#define GRAPH_Y_0 309
#define GRAPH_X_SPAN 237
#define GRAPH_Y_SPAN 194
#define GRAPH_PWM_SPAN 400 // in PWM - (0 - x)% for VNH5019 driver usually 0-400
#define GRAPH_PRESSURE_SPAN 10 //in bar: 0-x bar usually 0-10 bar
#define GRAPH_FLOW_VOLUME_SPAN 100 //in ml/min usually 0-90ml
#define GRAPH_FLOW_RATE_SPAN 400 //in ml/min usually 0-400ml/min


//***********************************************************************
// Color choices
//***********************************************************************
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

char* PWM_Color = ILI9341_RED;
char* PWM_BGColor = ILI9341_MAROON;
char* pressure_Color = ILI9341_CYAN;
char* pressure_BGColor = ILI9341_DARKCYAN;
char* flowVolume_Color = ILI9341_GREEN;
char* flowVolume_BGColor = ILI9341_DARKGREEN;
char* flowRate_Color = ILI9341_YELLOW;
char* flowRate_BGColor = ILI9341_GREENYELLOW;
char* weight_Color = ILI9341_ORANGE;
char* timer_Color = ILI9341_YELLOW;
char* text_light_Color = ILI9341_WHITE;
char* text_dark_Color = ILI9341_WHITE;
char* bg_Color = ILI9341_BLACK;
char* axis_minor_Color = ILI9341_LIGHTGREY;
char* axis_major_Color = ILI9341_WHITE;

//****************************************************************************
// Graph functions
//***************************************************************************
  
void drawGraphPixels(uint16_t pumpPWM, int profileIndex, float averagePressure, uint16_t long flowPulseCount, bool isBackground, bool preInfusion)
{
	char* colorPWM = PWM_Color;
	char* colorPressure = pressure_Color;
	char* colorFlowVolume = flowVolume_Color;
	char* colorFlowRate = flowRate_Color;
	
	if (isBackground) // draws profile in background (muted) colors
	{
		colorPWM = PWM_BGColor;
		colorPressure = pressure_BGColor;
		colorFlowVolume = flowVolume_BGColor;
		colorFlowRate = flowRate_BGColor;
	}

    // Draw 3 current graph pixels
	uint16_t x = graph_x(profileIndex);
	uint16_t y = graph_y(pumpPWM * 10, GRAPH_PWM_SPAN * 10); // * 10 to increase resolution
	tft.fillRect(x, y, 2, 2, colorPWM);
	y = graph_y(averagePressure * 100, GRAPH_PRESSURE_SPAN * 100); // * 10 to increase resolution to .01 bar
	tft.fillRect(x, y, 2, 2, colorPressure);
	y = graph_y(flowVolume() * 10, GRAPH_FLOW_VOLUME_SPAN * 10);
	tft.fillRect(x, y, 2, 2, colorFlowVolume);
	y = graph_y(flowRate(preInfusion) * 10, GRAPH_FLOW_RATE_SPAN * 10);
	tft.fillRect(x, y, 2, 2, colorFlowRate);
}

uint16_t graph_y(float y, uint16_t value_span)
{
	uint16_t y_out = map(y, 0, value_span, 0 , GRAPH_Y_SPAN); 
	y_out = constrain(GRAPH_Y_0 - 1 - y_out, GRAPH_Y_0 - GRAPH_Y_SPAN, GRAPH_Y_0 - 1);
	return y_out;
}

uint16_t graph_x(uint16_t profileIndex)
{
	uint16_t delta_x = round(MAX_PROFILE_INDEX / GRAPH_X_SPAN); 
	profileIndex = profileIndex * delta_x;
	return GRAPH_X_0 + 1 + profileIndex;
}

void printSomething(char* text, unsigned x, unsigned y, char* selectedColor, const GFXfont* selectedFont, boolean clearBackground)
{
	// How to concatenate chars?
	tft.setTextColor(selectedColor);
	tft.setFont(selectedFont);
	if (clearBackground)
	{
		int  x1, y1;
		unsigned w, h;
		tft.getTextBounds(text, x, y, &x1, &y1, &w, &h);
		tft.fillRect(x1, y1, w, h, bg_Color);
	}
	tft.setCursor(x, y);
	tft.print(text);
}  
  
void graphDrawEmptyGraph()
{
	int x0 = GRAPH_X_0;
	int y0 = GRAPH_Y_0;
	int x1 = GRAPH_X_0 + GRAPH_X_SPAN;
	int y1 = GRAPH_Y_0 - GRAPH_Y_SPAN;
	
	clearGraphArea();

	// Write the axis labels
	printSomething("--100%", 135, 115, PWM_Color, NULL , false);
	printSomething("--10bar", 135, 126, pressure_Color , NULL , false);
	printSomething("--100ml", 185, 115, flowVolume_Color, NULL , false); //-23
	printSomething("--400ml/m", 185, 126, flowRate_Color , NULL , false);
	//printSomething("0", 23, 258, text_light_Color, NULL , false);
	printSomething("Time", 110, 313, text_light_Color, NULL , false);
	printSomething("120s", 215, 311, text_light_Color, NULL , false);
	printSomething("0s", 1, 311, text_light_Color, NULL , false);

	// draw the axis
	//tft.drawFastHLine(1,262,239,axis_major_Color);
	tft.drawFastHLine(x0 - 2, y0 , x1 - x0 + 2,axis_major_Color);
	tft.fillTriangle(240,y0,236,311,236,307,axis_major_Color);
	tft.drawFastVLine(x0,y1,194,axis_major_Color); //134-115 = 19
	tft.fillTriangle(x0,y1,x0 - 2,119,x0 + 2,119,axis_major_Color);
  
  void clearGraphArea()
{
	tft.fillRect(GRAPH_X_0 + 1, GRAPH_Y_0 - GRAPH_Y_SPAN, GRAPH_X_SPAN, GRAPH_Y_SPAN, bg_Color); //redrawing only the graphing area so as to eliminate axis flicker......	
}
