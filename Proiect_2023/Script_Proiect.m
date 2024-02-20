clc, clear, close all

% Definirea Parametrilor
param.m = 5;     % masa obiectului
param.l0 = 1.4;  % lungimea initiala
param.csi = 0.9; % factor amortizare
param.k = 0.75;  % constanta de elasticitate
param.g = 9.81;  % acceleratia gravitationala

% Definire bus
par_bus_info = Simulink.Bus.createObject(param);
par_bus = evalin('base', par_bus_info.busName);

%% Cerinta 1
% Incarcare model, cerintele a) si b)
load_system('Model_Simulink_Proiect.slx')

%% Cerinta 2
% Definirea timpului pentru simulare
T = 100;
set_param('Model_Simulink_Proiect', 'StopTime', num2str(T));

% Generam intrarea
t = linspace(0,T,1000);
st = double(t>=0);
usim = timeseries(st,t);

% Simulam modelul
out = sim('Model_Simulink_Proiect');

% Afisez graficele pentru a verifca daca ajung in regim stationar
figure('Name', 'Grafice pentru verificarea regimului stationar');
subplot(2,1,1);
plot(out.simout_x.time, out.simout_x.data, 'b', 'LineWidth', 1.5);
title('Grafice pentru x')
xlabel('Timp (s)');
ylabel('X (cm)');
hold on
subplot(2,1,2);
plot(out.simout_theta.time, out.simout_theta.data, 'r', 'LineWidth', 1.5);
title('Grafice pentru theta')
xlabel('Timp (s)');
ylabel('Theta (rad)');

%% Cerinta 3
% Simulam modelul
out = sim('Model_Simulink_Proiect');

simout_mat1 = out.simout_x.data;
simout_fcn1 = out.simout_fcn1.data;

simout_mat2 = out.simout_theta.data;
simout_fcn2 = out.simout_fcn2.data;

% Verificare grafic pentru X
figure('Name', 'Verificare grafica pentru X');
plot(out.simout_x.time, simout_mat1, 'r', 'LineWidth', 2);
title('X')
hold on;
plot(out.simout_fcn1.time, simout_fcn1, 'g', 'LineWidth', 1);

% Verificare grafic pentru Theta
figure('Name', 'Verificare grafica pentru Theta');
plot(out.simout_theta.time, simout_mat2, 'r', 'LineWidth', 2);
title('Theta')
hold on;
plot(out.simout_fcn2.time, simout_fcn2, 'g', 'LineWidth', 1);

% Se poate observa ca in ambele cazuri, cele 2 grafice sunt identice, 
% au aceleasi valori

%% Cerinta 4
% Calcularea norma 2
err1 = norm(simout_mat1 - simout_fcn1);
err2 = norm(simout_mat2 - simout_fcn2);
disp('Eroarea dintre simout_mat1 si simout_fcn1');
disp(err1);
disp('Eroarea dintre simout_mat2 si simout_fcn2');
disp(err2);

% Asteptam sa obtin valori mici pentru err1 si err2, ceea ce s-a intamplat
% Aceste valori le-am obtinut deoarece graficele celor 2 aproape coincid

%% Cerinta 5
% Cerinta a)
ustar = linspace(0.05, 3, 10);
y1star = zeros(10, 1);
y2star = zeros(10, 1);

% Cerinta b)
for i = 1:10
    in = ustar(i) .* double(t>=0);
    usim = timeseries(in,t);

    out = sim('Model_Simulink_Proiect');
    y1star(i) = simout_mat1(end);
    y2star(i) = simout_mat2(end);
end
disp('Ustar');
disp(ustar);
disp('Y1star');
disp(y1star);
disp('Y2star');
disp(y2star);

% Cerinta c)
% Aproximare polinomiala ordin 3
p1_ord3 = polyfit(ustar, y1star, 3);
p2_ord3 = polyfit(ustar, y2star, 3);

ustar1 = linspace(ustar(1), ustar(end), 100);
y1starr_ord3 = polyval(p1_ord3, ustar1);
y2starr_ord3 = polyval(p2_ord3, ustar1);

% Cerinta d)
figure('Name', 'Caracteristica statica de functionare');
subplot(2,1,1);
plot(ustar, y1star, 'xk');
hold on
plot(ustar1, y1starr_ord3, 'LineWidth', 1.5);
xlabel('u^*'), ylabel('y^*');
legend('Simulink', 'Aproximare polinomiala ordin 3');
title('Grafic y1star');

subplot(2,1,2);
plot(ustar, y2star, 'xk');
hold on
plot(ustar1, y2starr_ord3, 'LineWidth', 1.5);
xlabel('u^*'), ylabel('y^*');
legend('Simulink', 'Aproximare polinomiala ordin 3');
title('Grafic y2star');

%% Cerinta 6
% Alegerea valorilor
alfa = 3.25;
beta = 4.14;
gamma = 4.87;

% Crearea noilor variabile pentru a verifica aproximarea
ustar6 = [alfa, beta, gamma];
y1star6 = polyval(p1_ord3, ustar6);
y2star6 = polyval(p2_ord3, ustar6);

figure('Name', 'Verificare aproximare');
subplot(2,1,1);
plot(ustar, y1star, 'xk');
hold on
plot(ustar6, y1star6, 'LineWidth', 1.4);
xlabel('u^*'), ylabel('y^*');
legend('Simulink', 'Aproximare polinomiala ordin3');
title('Grafic y1star');

subplot(2,1,2);
plot(ustar, y2star, 'xk');
hold on
plot(ustar6, y2star6, 'LineWidth', 1.4);
xlabel('u^*'), ylabel('y^*');
legend('Simulink', 'Aproximare polinomiala ordin3');
title('Grafic y2star');

%% Cerinta 7
% Am creat modelul "Model2_inout.slx" asemanator celui de la cerinta 1
% Exista o singura diferenta, inlocuirea blocurilor
% FromWorkspace/ToWorkspace cu blocurile Input/Output

%% Cerinta 8
%Valoarea intrarii pentru care se determina Punctul Static de Functionare
u0 = ustar(1);                                                                  
%Determinam Punctul Static de Functionare
[xstarr, ustarr, ystarr, ~] = trim("Model2_inout", [], u0, [], [], 1, []);      

err = norm(abs(ustarr - u0));
disp('Eroarea dintre ustarr si u0:');
disp(err);
% Nu exista diferente intre cele doua valori,
% asa cum asteptam, in functia trim am setat ca intrarea sa fie fixata

%% Cerinta 9
%Liniarizarea in Punctul Static de Functionare anterior
[A_lin, B_lin, C_lin, D_lin] = linmod("Model2_inout", xstarr, ustarr);       

%% Cerinta 10
% Sistemul liniarizat este stabil deoarece toate valorile proprii ale lui
% A_lin au partea reala negativa
vp = eig(A_lin);
disp('Valorile proprii A_lin:');
disp(vp);

%% Cerinta 11
load_system('Model3_raspLin')
usim = timeseries(st, t);                                                        
set_param('Model3_raspLin', 'StopTime', num2str(T))                                      
out = sim('Model3_raspLin');                                                                
y_lin = out.y_lin;

%% Cerinta 12
y_nlin = out.y_nlin;

y_lin_reshaped = reshape(y_lin.data, [1,66]);
y_nlin_reshaped = reshape(y_nlin.data, [1,66]);

err = norm(y_lin_reshaped - y_nlin_reshaped, 'inf');
disp('Eroarea dintre iesirea liniara si iesirea neliniara');
disp(err);

%% Cerinta 13
[b, a] = ss2tf(A_lin, B_lin, C_lin, D_lin);
H_lin = tf(b, a);
Te = 0.09;                                                                                  %Perioada de esantionare de 0.04s
H_disc = c2d(H_lin, Te, 'tustin')

% Am ales aproximarea Tustin deoarece sistemul este stabil, aam demonstrat
% la cerinta 10. Aproximarea Tustin plaseaza polii stabili ai sistemului 
% continuu in polii stabili ai sistemului discret. 
% Astfel, sistemul discretizat va fi si el stabil.

%% Cerinta 14











