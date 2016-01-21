/* The following code will return the required speed you
 * will need to shoot at in order to score a ball in the goal.
 * The input to the function is the distance that you are away
 * from the goal.
 *
 * The following are the properties specific to this code:
 * Launcher Height: 1.0833 feet.
 * Launcher Angle: 23 degrees.
 *
 * Distance is measured in feet.
 * Speed is measured in feet per second.
 *
 * The valid range of distances you can enter are from
 * 10.25 feet to 16.5417 feet and the spacing between
 * values is 0.0417 feet.
 *
 * Values less than the minimum distance or greater than
 * the maximum distance will return a value of -1.
 */

static float TRJ_firingTable[] = {
	31.719520044852,
	31.720621236397,
	31.721231234595,
	31.723598841019,
	31.724311523034,
	31.727883946654,
	31.732036888832,
	31.736756332312,
	31.742028683724,
	31.743310801449,
	31.749649763908,
	31.756503250153,
	31.763859210375,
	31.771705946048,
	31.780032097259,
	31.788826630598,
	31.798078827552,
	31.798718115878,
	31.817914846586,
	31.819418629882,
	31.839460293746,
	31.841790352152,
	31.862639682782,
	31.865759869682,
	31.869262282859,
	31.891257712156,
	31.895499645309,
	31.918218092465,
	31.92316800156,
	31.946578667522,
	31.952206644822,
	31.976280317524,
	31.982557987715,
	31.989150517709,
	32.014167366173,
	32.021369709933,
	32.04698285799,
	32.054770065785,
	32.062840982533,
	32.089303463102,
	32.116037205936,
	32.124924124072,
	32.134072484373,
	32.143477061373,
	32.171243578338,
	32.19925502097,
	32.209396887348,
	32.219775255317,
	32.248493802544,
	32.277438363737,
	32.288497504598,
	32.299775218251,
	32.311267284738,
	32.341074442048,
	32.371086340534,
	32.383195612687,
	32.395503270089,
	32.408005548736,
	32.456900958948,
	32.469779943503,
	32.482842728044,
	32.496085870499,
	32.509506003843,
	32.559293918067,
	32.573056536822,
	32.586986455543,
	32.601080588444,
	32.615335915413,
	32.665934935308,
	32.680502060022,
	32.695221677102,
	32.71009101299,
	32.761285558827,
	32.776444383874,
	32.791744929946,
	32.807184645923,
	32.822761032779,
	32.874640393724,
	32.89048088649,
	32.90645083708,
	32.922547942618,
	32.938769946234,
	32.991273527756,
	33.007736718133,
	33.024318286572,
	33.041016149685,
	33.093979001424,
	33.110901291972,
	33.127933854213,
	33.145074762339,
	33.162322127595,
	33.215814332152,
	33.233266950282,
	33.250820560745,
	33.268473414862,
	33.286223796929,
	33.340199441234,
	33.358137661447,
	33.376168441663,
	33.39429019102,
	33.412501348081,
	33.466918692172,
	33.485301841152,
	33.503769873264,
	33.522321338619,
	33.577064049222,
	33.595775841854,
	33.614566863996,
	33.633435767851,
	33.652381229734,
	33.707499618465,
	33.726591978081,
	33.745757055381,
	33.764993618278,
	33.820388707615,
	33.839762255699,
	33.859203712972,
	33.878711931467,
	33.898285783117,
	33.954000448953,
	33.973699844111,
	33.993461595988,
	34.01328465228,
	34.069234557989,
	34.089174691585,
	34.109173071781,
	34.129228716047,
	34.149340658386,
	34.169507949026,
	34.225781452388,
	34.246054166401,
	34.266379467021,
	34.322800783927,
	34.343226102382,
	34.363701381168,
	34.384225775584,
	34.40479845478,
	34.425418601453,
	34.4821146023,
	34.502824743573,
	34.523579973711,
	34.5804010771,
	34.601241643045,
	34.622125032818,
	34.64305051727,
	34.664017378875,
	34.685024911516,
	34.742078546578,
	34.76316275957,
	34.784285586883,
	34.841444714295,
	34.862640191296
};

float TRJ_getFiringTable(float distance) {
	int index = (distance - 10.25) / 0.041666666666667;

	if(index >= 0 && index < 152) {
		return TRJ_firingTable[index];
	}

	return -77;
}
