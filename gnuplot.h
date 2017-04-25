
//gnuplot API

#define WINDOWS 1

void plotCSV(){
  // /*
  #if WINDOWS
    FILE* gnuplot = _popen("C:/gnuplot/bin/pgnuplot.exe", "w");
  #else
    FILE* gnuplot = popen("gnuplot", "w");
  #endif
  	fprintf(gnuplot, "set datafile separator ','\n");
  	fprintf(gnuplot, "plot 'Test.csv' using 1 w l, 'Test.csv' using 2 w l , 'Test.csv' using 3 w l, 'Test.csv' using 4 w l \n");
  	fprintf(gnuplot, "plot 'Test.csv' using 1 w l, 'Test.csv' using 2 w l , 'Test.csv' using 3 w l, 'Test.csv' using 4 w l, 'Test.csv' using 5 w l \n");
  	// fprintf(gnuplot, "plot 'Test.csv' using 1 w l,'Test.csv' using 3 w l, 'Test.csv' using 4 w l \n");
  	// fprintf(gnuplot, "plot '~/Test.csv' using 5 w l, '~/Test.csv' using 6 w l\n");
    fflush(gnuplot);

  #if WINDOWS
    system("pause");
    fprintf(gnuplot, "exit\n"); // gnuplotの終了
  	_pclose(gnuplot);
  #endif
  // */
}
