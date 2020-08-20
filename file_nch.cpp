#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
using namespace std;

int main(){
	
	
	ifstream read_;
	ofstream write_;
	cout<<"파일 변환을 시작합니다."<<endl;
	



	
	for(int i=1229;i<=2681;i++){
skip00:	string z ="/home/lee_yeong_hyeon/Desktop/img/2020.08.07.SL_3lamp3-3_0000"+to_string(i)+".txt";
		

	
		
			
		read_.open(z,ios::in|ios::ate);
		
		if(read_.fail()){
			cout<<"read fail"<<endl;
			i++;
			read_.close();
			goto skip00;
		}
	
		int size = read_.tellg();
		
		
		
		cout<<size<<endl;	
		read_.seekg(0);
		char str[size]={};
		
			
		int j = 0;
		if(read_.good())
		{
				
				read_.getline(str, size);
				
				write_.open(z);
				for(int k=0; k < (size-1); k++){
					
					
					if((str[k] == '1'&& str[k+1] == '1') && (j != 1)){
						//str[k] = '1';
						str[k+1] = '0';
						write_<<str[k+1];
						j = 1;
						
						}
					else if((str[k] == '1' && str[k+1] =='2') && (j != 1)){
						//str[k] = '2';
						str[k+1] = '1';
						write_<<str[k+1];
						j = 1;
					}
					else if((str[k] == '1' && str[k+1] == '3') && (j != 1)){
						//str[k] = '3';
						str[k+1] = '2';
						write_<<str[k+1];
						j = 1;
					}

					else if((str[k] == '1' && str[k+1] == '4') && (j != 1)){
						//str[k] = '3';
						str[k+1] = '3';
						write_<<str[k+1];
						j = 1;
					}
					else if(k > 1 && j == 1)
						write_<<str[k];
					else if(j != 1)
						write_<<str[k];
				}
				
				
		}

	read_.close();
	write_.close();
	if( j != 0)
		cout<<z<<" 문서를 변환 하였습니다."<<endl;
	}
	
	cout<<"  **END**  "<<endl;	
	
	return 0;
}


