program adccontrol_new;

{$mode objfpc}{$H+}
{ PC side for high res MS ADC based on Mega48
  read data via RS232, convert to voltage and save to file
  keyboard commands send directly
 }

 uses
   {$IFDEF UNIX}{$IFDEF UseCThreads}
   cthreads,
   {$ENDIF}{$ENDIF}
   Classes, SysUtils, Synaser, Crt  ;

 var
   ser       : TBlockSerial;   { seriell interface }

const
 m = 20;        { output to screen every m results}
 out_mode = 3;   //  output format (not yet fully implemented)
                 // 0 -> only average of result
                 // 1 -> only average
                 // 2 -> u1,u2,...
                 // 3 -> raw

   { constants depending on ADC board (HW and software) 2. bord:
   }
   adcclock = 12000000;     // clock on ADC board
   scale = 7000;            //  Ref voltage in mV
   xd = 12*3;               //  extra  delay 3* xdel in ASM code
   k1 =  1.0 / 21.185;      //  measured ref ratios from adjustment
   k2 =  4.0/89.2;          //  fine step / adc LSB


   { constants depending on ADC board (HW and software) 1. board:
     adcclock = 16000000;      // clock on ADC board
     scale = 7020;             // Ref voltage in mV
     xd = 0*3;                 // extra  delay 3x xdel in ASM code
     k1 =  1.0 / 19.4337;   // measured ref ratios from adjustment
     k2 =  4.0/ 143.3;  // fine step / adc LSB
   }

 { Lenght of active run-up phase for different RU mode P,Q,R,S,T,U,V,W, has to match AVR software: }
  k0: array [0..7] of integer=(51+xd-16,102+2*xd-24,102+2*xd-16,102+2*xd-16,102+2*xd-16,102+2*xd-24,102+2*xd-36,204+4*xd-24);

  k1_puls = 25 ; { lentgh of long K1 pulse, needed for skale factor measurement, should be 25 }
                 { may need to be smaller if integrator capacity is small }
  sf0 =  (2-k1) * scale  / (adcclock * 0.02) ;
      { crude skaling units -> uV  ,depends on HW (clock + ref) }
      { run mode C for some  time to get accurate scale factor  }

  m_sf = 200;   {averaging for scale factor usually 100...500}


Var     k,c : char;
        n,outcnt,ruv : word;
	rawind   : word;
        f, fkom  : text;
	u,u1,u2,u3,u4,du,u2old : double;   { voltages }
	ru : word;     { runup }
	adc1, adc2, adcalt,adcdiff : longint;
        raw : array [0..60] of longint;
	sum, sum5,sum25 : longint;
	n5,n25 : word;
	sumA, sumB : longint;
        sumdu,avdu,sumq,rms,sf :  double;
        sumu1,sumu2,sumu3,u1m,u2m,u3m : double;
        sumk1,sumk2, sumsf : double;
        countk,countsf : integer;


function checkscreen: boolean;  { sum up and check if ready for output }
var avduold :double;
begin
 outcnt := outcnt +1;
 sumdu := sumdu+du;
 sumq := sumq + sqr(du-avdu);
 sumu1 := sumu1 + u1;
 sumu2 := sumu2 + u2;
 sumu3 := sumu3 + u3;
 if outcnt = m then
  begin 
   outcnt := 0;
   checkscreen := true;
   avduold := avdu;
   avdu := sumdu/m;
   rms := sqrt((sumq - sqr(avdu-avduold)) / (m-1));
   u1m := sumu1/m;
   u2m := sumu2/m;
   u3m := sumu3/m;
   sumu1 := 0;
   sumu2 := 0;
   sumu3 := 0;

   sumdu := 0;
   sumq := 0;
  end
 else
 checkscreen := false;
end ;
		

Procedure writeStatus;
 begin
     Write('Device: '          + ser.Device +
           ' Windows Status: ' + ser.LastErrorDesc +
           ' Error Code '      + Inttostr(ser.LastError));
     writeln;
 end;

function Readcom : char;
 begin
     // write('*');  { for debug }
  repeat
  until ser.canread(0);         // wait for char
  Readcom := chr(ser.RecvByte(0));       // get 1 byte
 end;

function read8 :word;       { read value and remember}
VAR  k : word;
begin
  k :=ord(Readcom);
  raw[rawind] := k;         { remember raw values as words }
  rawind := rawind +1;
  read8 := k;
end;
        		
function read16 :word;     
Var       k,k2  : word;
begin
  k :=ord(Readcom);         { low byte }
  k2:=ord(Readcom);
  k := 256 * k2 + k; 
  raw[rawind] := k;         { remember raw values as word }
  rawind := rawind +1;
  read16 := k;
end;

function read24 :longint;
Var       k,k1,k2  :longint;
begin
  k :=ord(Readcom);         { low byte }
  k1:=ord(Readcom);
  k2:=ord(Readcom);
  k := k + 256 * (k1 + 256 * k2); 
  raw[rawind] := k;         {remember raw values}
  rawind := rawind +1;
  read24 := k;
end;
  
function readADC(k_0:double):double ;  { read one ADC result }
 { k_0 is lenght of runup steps in cycle  }
 { result is in cycles of small (negative) reference }
var  small, large, ru0 :longint ;
begin
   ru := read16;     { run-up counts}
   small := read16;  { cycles of weaker ref (negative}
   large := read16;  { cycles of stronger ref (pos}
    adc1 := read16;  { aux ADC , average integrator voltage, for info only }
    adc1 := read16;  { residual charge after conversion }
    adc2 := read16;  { residual charge before conversion }
    adcdiff := adc2-adc1;
    ru0 := round(adcclock *0.02 / (k_0+16) / 2);   { approximate zero : length K_o+16 is not accurate }
    readADC := (k_0 *(ru-ru0)) *(2+k1) + (small - large*(1+k1) + (adcdiff)*k1*k2) ;
    adcalt:= adc2;   { remember old res. charge (for version with only 1 reading)}
end;

procedure writetime(var f:text);
 { write date an time to file + info on konstants }
var time : TSystemTime ;
begin
   SysUtils.GetLocalTime(time);
   write(f, time.month,'/',time.day,' ', time.Hour,':',time.Minute);
   write(f,' with k0=',k0[1]);
   write(f,' k1=',1/k1:6:4);
   writeln(f,' k2=',4/k2:6:2);
end;


Procedure Fileopen;
VAR     fn,kom : string;
        
begin
   fn := '';
   write('Filename '); Readln(fn);
   if fn = ''then fn := 'test.txt' ;
   write('Kommentar '); Readln(kom);

   assign(f,fn);
   rewrite(f);
   write(f,'# ');
   writetime(f);
   writeln(f,'# ',fn);
   Writeln(f,'# ',kom);
   if fn <> 't' then begin
    assign(fkom,'log_kom.txt');  { extra log file }
    append(fkom);
    writeln(fkom,fn,' ',kom);
    writetime(fkom);
    close(fkom);
   end;
end;   


Procedure writeLogK;
VAR  fkom : text;
begin
    assign(fkom,'log_kom.txt');
    append(fkom);
    write(fkom,'# K1,K2,SF= ',sumk1/countk:8:6,' ',sumk2/countk:8:2,' ', sf*1000:8:7,'  ');
    writetime(fkom);
    close(fkom);
    sumk1 := 0;
    sumk2 := 0;
    countK :=0;
end;

procedure writeraw;   { write raw data to file (end of line)}
Var  i   :  word;
begin
 for i  := 0 to rawind-1 do
    write(f,' ',raw[i]);
 writeln(f);
 rawind :=0;
end;

Procedure skalefactor1;  { result from slow slope measurement }
var  pulseL, adcl : integer;  { sums are global }
begin
 sum := read24;      { sum of slow slope lenght }
 pulseL := read8;    { pulse length }
 if pulseL = 5 then  { sum up data for short }
   begin
    sum5 := sum5 + sum;
    n5 := n5 + 1;
   end;
  if pulseL = k1_puls then  { long case }
   begin
     sum25 := sum25 + sum;
     n25 := n25 + 1;
   end;
  adcl := read16;   // dummy read not yet used, not needed
  adcl := read16;
       { write(f,n:4);     type of data}
       { writeraw;         raw data }
  end;

Procedure skalefactor2;  { result of ADC scale factor measurement}
var m1,m2  : integer;
begin
  m1 := read8;      { number of steps up }
  m2 := read8;      { number of steps down }
  sumA := read16;
  sumB := read16;
  adc1 := read16;   { for debug only, but need to read !}
  adc2 := read16;
  writeraw;
  if n5 > 2 then
   begin  { enough data from adjustK1 ( slow slope) }
     u := (sum25 /n25 - sum5 /n5);    { difference of average }
     u3 := u / (3*(k1_puls-5)*128);   { calculate slope ratio, include fixed constants }
     write(f,'; k1= ', u3:8:6);
     writeln(f);
     write('## k1 = ', u3:8:5);
     sum5 := 0;       {reset sums after use }
     sum25 := 0;
     n5 :=0;
     n25 := 0;
   end;
  if (m1 > 2) and (m2 >2) then          { useful result from ADC scale }
   begin
     u := sumA;         u := u/m1;
     u2 := 65536-sumB; u2 := u2/m2;     { U2 is negative }
     sumk1 := sumk1 + U3;
     sumk2 := sumk2 + (U+u2)/2;
     countk := countk +1;
     if countsf = m_sf then
      begin
        sf := sumsf / m_sf;
        writeln(' update scalefactor from ref reading', sf:8:5);
        sumsf :=0;      { reset sum to allow update}
        countsf :=0;
      end;
     if countk > 5 then  writeLogK;
     write(f,'; k2= ', (u+u2)/2:8:5, u:8:3 , u2:8:3,'  SF= ',sf*1000:8:5);
     writeln(f);
     writeln('## k2 : ', u:8:3 , u2:8:3 , m1:3,m2:3,(u+u2) / 2 :8:3);
   end
  else
   writeln(' Problem with ADC data: ',m1:10,m2:10, sumA:10,sumB:10); { invalid data }
end;


begin               { main program  }
  ruv := 1;         { default RU version}
  outcnt := 0;
  sumdu :=0;
  sumq :=0;
  avdu :=0;
  u2old:=0;
  rawind := 0;
  sumk1 := 0;
  sumk2 := 0;
  sumsf := 0;
  countk := 0;
  countsf := 0;
  sf := sf0;  { crude estimate for scale factor as a start }

  fileopen;  { open data file }

  ser := TBlockSerial.Create;
 try
    ser.Connect('COM1:');   //ComPort <= 9  check portnumber on PC
                { /dev/ttyUSB0   for linux }
    ser.config(9600, 8, 'N', SB1, False, False);
    Sleep(10);
    writeStatus;
    ser.Purge;

    ser.sendByte(ord('C'));  // call for mode C = AZ mode with ref
    k := ' ';     // key - space as not send
    Sleep(10);
    writeStatus;

    while (k <> 'X') and (ser.LastError = 0) do
      begin
        rawind := 0;   // new package, reset counter for raw data
        repeat
          c:=Readcom;     {wait for sync}
        until ord(c) = 255;
        c:=Readcom;     { get tag }
        n := ord(c);     // for debuging:   write(n,'  ');

       case n of      // action depending on tag
       254, 251 :                  // 2 readings (modes A, B, E)
         begin
          u1 := readADC(k0[ruv]);    { result of 1 st conversion }
          if n=254 then
            u2 := readADC(k0[ruv])    { result of 2. conversion }
           else
            u2 := readADC(k0[1]);     { result of 2. conversion, mode B for INL test }
          du:=u1-0.5*(u2+u2old);

          case out_mode of
          3: begin    // full data with raw
              write(f,' ',u1:8:3,' ',u2:8:3,' ',du:8:3,' ', adcdiff:6);
              writeraw;
              if checkscreen then
                 writeln( ru,' ',u1m:8:3,' ',u2m:8:3,' ',du:8:3, ' ', sf*avdu:9:4,' ',adc1:6, rms*sf:8:5);
             end;
          2: begin    // full data without raw
              writeln(f,' ',u1:8:3,' ',u2:8:3,' ',du:8:3,' ', adcdiff:6);
              if checkscreen then
                 writeln( ru,' ',u1m:8:3,' ',u2m:8:3,' ',du:8:3, ' ', sf*avdu:9:4,' ',adc1:6, rms*sf:8:5);
             end;
          1: begin    // only average
              if checkscreen then
                begin
                 writeln( ru,' ',u1m:8:3,' ',u2m:8:3,' ',du:8:3, ' ', sf*avdu:9:4,' ',adc1:6, rms*sf:8:5);
                 writeln( f,u1m:8:3,' ',u2m:8:3,' ',du:8:4, ' ', sf*avdu:9:5,' ', rms*sf:8:5);
                end;
              end;
           end;    // end case out_mode
           u2old := u2;
          end;
      250 :
        begin {3 readings }
         u1 := readADC(k0[ruv]);    { result of 1 st conversion }
         u2 := readADC(k0[ruv]);    { result of 2. conversion }
         u3 := readADC(k0[ruv]);    { result of 3. conversion }
         if (abs(u3-u2) >1000) then
           begin
            du:=(u1-u2)/(u3-u2);
            if countsf < m_sf then begin sumsf:=sumsf + scale/(u3-u2); countsf := countsf +1; end;
                 { sum up the first m_sf scale factor readings }
           end
         else du := (u1-u2) *sf/ 1000;    { approx scale if no valid 7 V }
         write(f,' ',u1:8:3,' ',u2:8:3,' ',u3:8:3,' ',du*scale:8:4,' ', adc1-adc2:6);
         writeraw;
         if checkscreen then
           writeln(ru,' ',u1m:8:3,' ',u2m:8:3,' ',u3m:8:3,' ',avdu*scale:9:4, adc1:6,' ',rms*scale:8:5);
         u2old := u2;
        end;
     253: skalefactor1;    { slope ratio measurement}
     252: skalefactor2;    { ADC skale factor and ouput of slope ratio }


    248,247:
     begin {4 readings }
       u1 := readADC(k0[ruv]);    { result of 1 st conversion }
       u2 := readADC(k0[ruv]);    { result of 2. conversion }
       u3 := readADC(k0[ruv]);    { result of 3. conversion }
       u4 := readADC(k0[ruv]);    { result of 4. conversion }
       if (abs(u4-u3) >1000) then du:=(u2-u3)/(u4-u3)
       else du := (u2-u3) *sf/1000.0;    { approx scale if no valid 7 V }
       write(f,' ',u1:8:3,' ',u2:8:3,' ',u3:8:3,' ',u4:8:3,' ',du*scale:8:4,' ', adc1-adc2:6);
       writeraw;
       if checkscreen then
         writeln(ru,' ',u1m:8:3,' ',u2m:8:3,' ',u3m:8:3,' ',u4:8:3,' ',avdu*scale:9:4, adc1:6,' ',rms*scale:8:5);
       u2old := u2;
     end;
   end;  // case

   if keypressed then
    begin
     k := upcase(readkey);
     ser.sendByte(ord(k));
    end;
   if (k >= 'P') and (k <= 'W') then ruv := ord(k)-80;         { update runup version }

  end; // loop to receive & send data.

  finally
      close (f);
      Writeln('Serial Port will be freed...');
      writeStatus;
      ser.free;
      Writeln('SerialPort was freed!');
      writeStatus;
  end;

end.
