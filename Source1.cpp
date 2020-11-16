//#include <iostream>
//#include <string>
//#include <vector>
//#include <algorithm>
//#include <cmath>
//using namespace std;
//inline void keep_window_open() {char ch; cin>>ch;}
/*
int main() {
 char friend_sex = 0;
 int age {};
 cout << "Please, enter name adressat: ";
 string first_name;
 cin >> first_name;
 cout << "Enter name Your friend: ";
 string friend_name;
 cin >> friend_name;
 cout << "Enter m if your friend male, else f (for female) ";
 cin >> friend_sex;
 cout << "Enter age adresata: ";
 cin >> age;
 while (age <= 0 || age >= 110) {
  cout << ("\nThis is joke! Enter true!");
  cin >> age;
 }
 cout << "\nDear " << first_name << ",\n";
 cout << "How are you? ";
 cout << "I have good. ";
 cout << "I miss you!";
 cout << "\nSo, you so far miss " << friend_name << "?";
 if (friend_sex == 109){
  cout << "\nIf you see " << friend_name << ", please, tell him "
                                          "call me";
 }
 else
  cout << "\nIf you see " << friend_name << ", please, tell her "
                                          "call me";
 cout << "\nI hear, you had birthday and you have now " << age << " age!";
 if (age < 12)
  cout << "\nThe next year you have " << age+1 <<" years old";
 if (age == 17)
  cout << "\nYou can in next year enter vote";
 if (age >= 70)
  cout << "\nI hope, what you not be bored in retired pay";
 cout << "\nYours, \n" << "\n";
 cout << "Igor";
}
*/
/*
int main(){
    double mile_1 = 1.609;
    double kilo{};
    double mile{};
    cout << "Enter, mile: ";
    cin >> mile;
    kilo = mile * mile_1;
    cout << "Yours KM: " << kilo << "km\n";
}
*/
/*
int main(){
    int val1{}, val2{};
    double d_val1{}, d_val2{};
    cout << "Enter two integral value: ";
    cin >> val1 >> val2;
    cout << "Enter two flout value: ";
    cin >> d_val1 >> d_val2;
    int res_min = min(val1, val2);
    int res_max = max(val1, val2);
    cout << "First variant to min max with func min/max()\n";
    cout << "Min_value: " << res_min << " " << "Max_value: " << res_max << "\n";
    cout << "Second varian to min max wis if/else\n";
    if (val1 > val2){
        cout << "Min_value: " << val2 << " ";
        cout << "Max_value: " << val1 <<"\n";
    }
    else {
        cout << "Min_value: " << val1 << " ";
        cout << "Max_value: " << val2 <<"\n";
    }
    cout << "Sum: " << val1 + val2 << "\n";
    double v3 = d_val1 * d_val2;
    cout << "d_v1 * d_v2: " << v3 << "\n";
    cout << "v1 * v2: " << val1 * val2 << "\n";
    double v4 = d_val1 / d_val2;
    cout << "d_v1 / d_v2: " << v4 << "\n";
    cout << "v1 / v2: " << val1 / val2 << "\n";
}
*/
/*
int main(){
    //int x{}, y{}, z{};
    string x = "", y = "", z = "";
    //cout << "Enter 3 integral value: ";
    cout << "Enter 3 word: ";
    cin >> x >> y >> z;
    if (x > y && x > z && y < z){
        cout << y << " "<< z << " " << x;
    } else {
        if (x < y && y < z) {
            cout << x << " " << y << " " << z;
        } else {
            if (z < x && x < y ) {
                cout << z << " " << x << " " << y;
            }else {
                if (z < x && x > y){
                    cout << z << " " << y << " " << x;
                    if (x < y && y < z){
                        cout << x << " " << y << " " << z;
                    }else {
                        if (x == y && x < z){
                            cout << x << "" << y << " " << z;
                        }else {
                            if (x == y && x > z) {
                                cout << z << " " << x << " " << y;
                            } else {
                                if (x == z && x < y) {
                                    cout << x << " " << z << " " << y;
                                } else {
                                    if (x == z && x > y) {
                                        cout << y << " " << x << " " << z;
                                    } else {
                                        if (y == z && x < y) {
                                            cout << x << " " << y << " " << z;
                                        } else {
                                            if (y == z && x > y) {
                                                cout << y << " " << z << " " << x;
                                            }else {
                                                cout << "Not have condition";
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
 */
 /*
 int main(){
     int number{};
      cout << "Enter integral number: ";
      cin >> number;
      if (number % 2 == 0){
          cout << "Number" << " " << number << " " << "is even\n";
      }
      else cout << "Number " << number << " is odd\n";
 }
  */
  /*
  int main(){
      string nul0 = "null";
      string one = "one";
      string two = "two";
      string three = "three";
      string four = "four";
      string vvod = "NULL";
      cout << "Enter (stop) for exit from program. Enter number is words: ";
      while (vvod != "stop") {
          cin >> vvod;
          if (vvod == nul0){
              cout << "0\n";
          }
          if (vvod == one){
              cout << "1\n";
          }
          if (vvod == two){
              cout << "2\n";
          }
          if (vvod == three){
              cout << "3\n";
          }
          if (vvod == four){
              cout << "4\n";
          }
          if (vvod != nul0 && vvod != one && vvod != two && vvod != three && vvod != four){
              cout << "I don't now this number!\n";
          }
      }
  }
   */
   /*
   int main(){
       double x{}, y{};
       string operation = " ";
       cout << "Enter your string operation.(Like this: + 100 3.4): ";
       cin >> operation >> x >> y;
       if (operation == "+" || operation == "plus"){
           cout << x + y << "\n";
       }
       if (operation == "-" || operation == "minus"){
           cout << x - y << "\n";
       }
       if (operation == "*" || operation == "mul"){
           cout << x * y << "\n";
       }
       if (operation == "/" || operation == "div"){
           double res = x / y;
           cout << res;
       }
   }
    */
    /*
    int main (){
        double cent1{},cent5{},cent10{},cent25{},cent50{};
        cout << "Enter how you have 1-cent, 5-cent, 10-cent, 25-cent, 50-cent: \n";
        cin >> cent1 >> cent5 >> cent10 >> cent25 >> cent50;
        if (cent1 == 1) {
            cout << "You have " << cent1 << " 1-cent coin\n";
        }
        else cout << "You have " << cent1 << " 1-cents coin\n";
        if (cent5 == 1) {
            cout << "You have " << cent5 << " 5-cent coin\n";
        }
        else cout << "You have " << cent5 << " 5-cents coin\n";
        if (cent10 == 1) {
            cout << "You have " << cent10 << " 10-cent coin\n";
        }
        else cout << "You have " << cent10 << " 10-cents coin\n";
        if (cent25 == 1) {
            cout << "You have " << cent25 << " 25-cent coin\n";
        }
        else cout << "You have " << cent25 << " 25-cents coin\n";
        if (cent50 == 1){
            cout << "You have " << cent50 << " 50-cent coin\n";
        }
        else cout << "You have " << cent50 << " 50-cents coin\n";
        double over = cent1 + (cent5*5) + (cent10*10) + (cent25*25) + (cent50*50);
        cout << "Over you have " << over << " cents\n";
        int div = 0;
        div = over/100;
        int div1 = div * 100;
        double div_cent = over - div1;
        cout << div << " dollar " << div_cent << " cents";
    }
     */
     /*
     #include <iostream>
     #include <string>
     using namespace std;
     int main() {
         constexpr double rub_per_dollar = 67.66;
         constexpr double gvn_per_dollar = 27.83;
         constexpr double uan_per_dollar = 6.95;
         constexpr double euro_per_dollar = 0.88;
         constexpr double pound_per_dollar = 0.78;
         double value = 0;
         char type_tran{};
         char unit{};
         cout << "Please, enter your how mach money"<< "\n";
         cin >> value;
         cout  << "Nation(r 'rub', g 'gvn', u 'uan', e 'euro', p 'gbp')";
         cin >> unit;
         while (unit != 'r' && unit != 'g' && unit != 'u' && unit !='e' && unit != 'p') {
             cout << "I don't now that your mean! " << unit << "\n";
             cout << "Enter nation: ";
             cin >> unit ;
         }
         cout << "Enter your type transaction(1 'dollar_to_" << unit << "' or 2 '" << unit << "_to_dollar')" << "\n";
         cin >> type_tran;
         while (type_tran != '1' && type_tran != '2') {
             cout << "I dont't now that your mean! " << type_tran << "\n";
             cout << "Enter typy: ";
             cin >> type_tran;
         }
         switch (unit){
             case 'r':
                 if  (type_tran == '1')
                     cout << value << "dollar == " << rub_per_dollar * value << "rub\n";
                 if (type_tran == '2')
                     cout << value << "rub == " << value / rub_per_dollar << "dollar\n";
                 break;
                 // trans gvn to dollar and anther
             case 'g':
                 if (type_tran == '1')
                     cout << value << "dollar == " << gvn_per_dollar * value << "gvn\n";
                 if (type_tran == '2')
                     cout << value << "gvn == " << value/gvn_per_dollar << "dollar\n";
                 break;
     // trans uan to dollar and anther
             case 'u':
                 if (type_tran == '1')
                     cout << value << "dollar == " << uan_per_dollar * value << "uan\n";
                 if (type_tran == '2')
                     cout << value << "uan == " << value/uan_per_dollar << "dollar\n";
                 break;
             case 'e':
                 if (type_tran == '1')
                     cout << value << "dollar == " << euro_per_dollar * value << "euro\n";
                 if (type_tran == '2')
                     cout << value << "euro == " << value/euro_per_dollar << "dollar\n";
                 break;
             case 'p':
                 if (type_tran == '1')
                     cout << value << "dollar == " << pound_per_dollar * value << "gbp\n";
                 if (type_tran == '2')
                     cout << value << "gbp == " << value/pound_per_dollar << "dollar\n";
                 break;
             default:
                 cout << "I don't understand";
                 break;
         }
         // trans rub to dollar and dollar to rub
      *
     // trans rub to dollar and dollar to rub
         if (unit == 'r' && type_tran == '1'){
             cout << value << "dollar == " << rub_per_dollar * value << "rub\n";
         }else if (unit == 'r' && type_tran == '2')
             cout << value << "rub == " << value / rub_per_dollar << "dollar\n";
     // trans gvn to dollar and anther
         if (unit == 'g' && type_tran == '1' && unit != 'r'){
             cout << value << "dollar == " << gvn_per_dollar * value << "gvn\n";
         }else if (unit == 'g' && type_tran == '2')
             cout << value << "gvn == " << value/gvn_per_dollar << "dollar\n";
     // trans uan to dollar and anther
         if (unit == 'u' && type_tran == '1' && unit != 'r') {
             cout << value << "dollar == " << uan_per_dollar * value << "uan\n";
         }else if (unit == 'u' && type_tran == '2')
             cout << value << "uan == " << value/uan_per_dollar << "dollar\n";
      *
     }
     */
     /*
     #include <iostream>
     #include <string>
     using namespace std;
     int square(int x){
         int n{};
         for (int i = 0; i < x; ++i) {
             n += x;
         }
         return n;
     }
     int main(){
         //char simbol {};
         //for(int number = 65; number <= 90; ++number){
         //    simbol = number;
         //    cout << simbol << "\t" << number << '\n';
         //}
         for (int i = 0; i<100; ++i){
             cout << i << '\t' << square(i) << '\n';
         }
     }
     */

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace std;
int main() {
    //vector<int> v = {5,7,9,4,6,8};
       //first type output
    //for (int i=0; i < v.size(); ++i)
    //    cout << v[i] << '\n';
       //second type output
    //for (int x : v)
    //    cout << x << '\n';
    vector<double> temps;
    for (double temp; cin >> temp;)
        temps.push_back(temp);
    double sum = 0;
    for (double x : temps) sum += x;
    cout << "Middle temp: "
        << sum / temps.size() << '\n';
    sort(temps.begin(), temps.end());
    cout << "Mediana temps: "
        << temps[temps.size() / 2] << '\n';
}

/*
#include <iostream>
#include <string>
#include <vector>

int main() {
   int a = 1;
   int b = 1;
   double c = a + b;
}

      */

