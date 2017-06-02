#include <qrReader.h>

qrReader::qrReader(){

}

qrReader::-qrReader(){

}

bool qrReader::find(Mat img){
	int skipRows = 3;
	int stateCount[5] = {0};
	int currentState = 0;
	for(int row = skipRows-1; row < img.rows; row += skipRows){
		stateCount[0] = 0;
		stateCount[1] = 0;
		stateCount[2] = 0;
		stateCount[3] = 0;
		stateCount[4] = 0;
		currentState = 0;
		uchar* ptr = img.ptr < uchar > (row);
		for(int col = 0; col < img.cols; col++){
			if(ptr[col] < 128){
				if((currentState & 0x1) == 1){
					currentState++;
				}
				stateCount[currentState]++;
			}else{
				if((currentState & 0x1) == 1){
					stateCount[currentState]++;
				}else{
					if(currentState==4){
						if(checkRatio(stateCount)){

						}else{
							currentState = 3;
							stateCount[0] = stateCount[2];
							stateCount[1] = stateCount[3];
							stateCount[2] = stateCount[4];
							stateCount[3] = 0;
							stateCount[4] = 0;
							continue;
						}
						stateCount[0] = 0;
						stateCount[1] = 0;
						stateCount[2] = 0;
						stateCount[3] = 0;
						stateCount[4] = 0;
						currentState = 0;
					}else{
						currentState++;
						stateCount[currentState]++;
					}
				}
			}
		}
	}
	return false;
}
bool qrReader::checkRatio(int stateCount[]){
	int totalFinderSize = 0;
	for(int i = 0; i < 5; i++){
		int count = stateCount[i];
		totalFinderSize += count;
		if(count == 0){
			return false;
		}
	}
	if(totalFinderSize<7){
		return false;
	}
	int moduleSize = ceil(totalFinderSize / 7.0);
	int maxVariance = moduleSize / 2;
	bool retVal = ((abs(moduleSize - (stateCount[0])) < maxVariance &&
		(abs(moduleSize - (stateCount[1])) < maxVariance) &&
		(abs(3 * moduleSize - (stateCount[2] < 3 * maxVariance) &&
		(abs(moduleSize - (stateCount[3])) < maxVariance) &&
		(abs(moduleSize - (stateCount[4])) <maxVariance));

	return retVal;
}
