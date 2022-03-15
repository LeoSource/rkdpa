clear
close all
clc

addpath('classes');
addpath(genpath('tools'));
dt = 0.005;
simu_mode = 'dynamics';
switch simu_mode
    case 'gravity'
        GravityIden(dt);
    case 'friction'
        FrictionIden(dt);
    case 'dynamics'
        CalRobotDynamics(dt);
end

%% robot complete dynamic calculation
function CalRobotDynamics(dt)
    base_params = [4.063586042,7.728312133,6.422796343,-2.032979531,...
            -0.16877069,-0.59609944,0.010400987,2.977483286,3.250381561,...
            -0.054543291,9.261866999,11.66750712,-1.070288068,0.150369777,...
            -0.236369341,0.117051341,1.41391292,2.025080279,-0.086374041,...
            8.915357117,7.620561437,0.327425184,-0.084042172,0.062158337,...
            -0.128045159,0.022200001,-0.128978911,0.136896835,4.445977886,...
            2.674181678,0.010132584,-0.026483749,0.02806931,-0.021066788,...
            0.216262962,0.11011495,-0.087359527,3.955445296,3.056393106,...
            0.067404172,-0.039217235,-0.02969273,-0.029639822,0.13131146,...
            -0.004378391,0.006743087,3.262977906,3.006167057]';
    filepath = 'C:/Default_D/leo/MSpace/XProject/src/data/';
    filename{1} = 'test_data_0101_121609.csv';
    filename{2} = 'dynamic_data_0108_124616.csv';
    filename{3} = 'dynamic_data_0108_125755.csv';
    filename{4} = 'dynamic_data_0108_130054.csv';
    filename{5} = 'dynamic_data_0108_130333.csv';
    jp_id = []; jv_id = []; ja_id = []; jt_id = [];
    for idx=1:3
        [jp,jv,ja,jt] = LoadDynIdenTraj([filepath,filename{idx}],1/dt,true);
        jp_id = [jp_id, jp]; jv_id = [jv_id,jv];
        ja_id = [ja_id,ja]; jt_id = [jt_id,jt];
    end
    for idx=1:size(jp_id,2)
        Hb = CalRegression(jp_id(:,idx),jv_id(:,idx),ja_id(:,idx));
        jt_validate(:,idx) = Hb*base_params;
    end
    
    for jidx=1:6
        figure
        plot(jt_id(jidx,:),'k'); hold on; plot(jt_validate(jidx,:),'r');
        title(['joint',num2str(jidx)]); grid on;
    end

end

%% robot gravity identification
function GravityIden(dt)
    [jpos,jvel,jtor,t] = LoadTestFile('./data/test_data_0110_092357.csv',dt);
    % PlotJointData(jpos,jvel,jtor,[1,2,3,4,5,6],[2,3,4,5],[2,3,4,5],t);
    rbtdef = CreateRobot();
    grav_iden = RobotDynamics(rbtdef);
    grav_iden.GravityIden(jpos,jtor);
    %%save gravity identification parameters%%
    time_tmp = datevec(now);
    time_stamp = [num2str(time_tmp(2),'%02d'),...
                            num2str(time_tmp(3),'%02d'),...
                            num2str(time_tmp(4),'%02d'),...
                            num2str(time_tmp(5),'%02d')];
    file_name = ['gravity/gravity_parameters_',time_stamp,'.txt'];
    dlmwrite(file_name,grav_iden.barycenter_params,'precision',12);
end

%% robot friction identification
function FrictionIden(dt)
    [~,jvel1,jtor1,~] = LoadTestFile('./data/test_data_0110_091511.csv',dt);% joint1 data
    [~,jvel2,jtor2,~] = LoadTestFile('./data/test_data_0110_091013.csv',dt);% joint5 data
    rbtdef = CreateRobot();
    rbtdyn = RobotDynamics(rbtdef);
    rbtdyn.FrictionIden(jvel1(:,1),jtor1(:,1),jvel2(:,5),jtor2(:,5));
    %%save friction identification parameters%%
    time_tmp = datevec(now);
    time_stamp = [num2str(time_tmp(2)),num2str(time_tmp(3)),num2str(time_tmp(4)),num2str(time_tmp(5))];
    file_name = ['gravity/friction_parameters',time_stamp,'.txt'];
    dlmwrite(file_name,rbtdyn.fric_params,'precision',12);
end

%% robot description
function rbt = CreateRobot()
    d1 = 0.048; a2 = 0.41; a3 = 0.41;
    d4 = 0.11; d5 = 0.08662; d6 = 0.035;
    mdh_table = [0, d1, 0, 0, 0, 0;...
                        0, 0, 0, -pi/2, 0, -pi/2;...
                        0, 0, a2, 0, 0, pi/2;...
                        0, d4, a3, 0, 0, -pi/2;...
                        0, d5, 0, -pi/2, 0, 0;...
                        0, d6, 0, pi/2, 0, 0];
    % pose_tool = SE3(rotx(-10), [0,0,0.116]);
    tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end

%% plot joint data
function PlotJointData(jpos,jvel,jtau,jpplot,jvplot,jtplot,t)
    figure;
    for idx=jpplot
        plot(t,jpos(:,idx),'DisplayName',['jpos',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('position(rad)'); hold on;
    end
    hold off; legend;
    figure;
    for idx=jvplot
        plot(t,jvel(:,idx),'DisplayName',['jvel',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('velocity(rad/s)'); hold on;
    end
    hold off; legend;
    figure;
    for idx=jtplot
        plot(t,jtau(:,idx),'DisplayName',['jtor',num2str(idx)]); grid on;
        xlabel('time(s)'); ylabel('torque(Nm)'); hold on;
    end
    hold off; legend;
end

%% calculate robot dynamic regressor
function Hb = CalRegression(q,dq,ddq )
    x0 = cos(q(2));
    x1 = sin(q(2));
    x2 = -x1;
    x3 = dq(1)*x2;
    x4 = dq(2)*x3;
    x5 = ddq(1)*x0 + x4;
    x6 = dq(1)*x0;
    x7 = dq(2)*x6;
    x8 = -x7;
    x9 = x1*x8;
    x10 = -ddq(1)*x1 + x8;
    x11 = x3*x6;
    x12 = ddq(2) + x11;
    x13 = ((x6)*(x6));
    x14 = ((dq(2))*(dq(2)));
    x15 = -x11;
    x16 = ((x3)*(x3));
    x17 = -x14;
    x18 = sin(q(3));
    x19 = -x18;
    x20 = cos(q(3));
    x21 = -x6;
    x22 = x18*x21 + x20*x3;
    x23 = dq(2) + dq(3);
    x24 = x22*x23;
    x25 = -x5;
    x26 = x19*x3 + x20*x21;
    x27 = dq(3)*x26 + x10*x20 + x18*x25;
    x28 = -x20;
    x29 = x23*x26;
    x30 = x27 + x29;
    x31 = -x24;
    x32 = -x10;
    x33 = -dq(3)*x22 + x18*x32 + x20*x25;
    x34 = x31 + x33;
    x35 = ((x23)*(x23));
    x36 = ((x22)*(x22));
    x37 = -x36;
    x38 = x35 + x37;
    x39 = x22*x26;
    x40 = ddq(2) + ddq(3);
    x41 = x39 + x40;
    x42 = -x35;
    x43 = ((x26)*(x26));
    x44 = x42 + x43;
    x45 = -x39;
    x46 = x40 + x45;
    x47 = 0.51*x32 + 0.51*x7;
    x48 = -x47;
    x49 = -x33;
    x50 = x24 + x49;
    x51 = cos(q(4));
    x52 = -x51;
    x53 = sin(q(4));
    x54 = x22*x53 + x26*x52;
    x55 = dq(4) + x23;
    x56 = x54*x55;
    x57 = x22*x51 + x26*x53;
    x58 = dq(4)*x57 + x27*x53 + x49*x51;
    x59 = x51*x56 + x53*x58;
    x60 = x52*x58 + x53*x56;
    x61 = x55*x57;
    x62 = x58 + x61;
    x63 = -x54;
    x64 = dq(4)*x63 + x27*x51 + x33*x53;
    x65 = -x56;
    x66 = x64 + x65;
    x67 = x52*x66 + x53*x62;
    x68 = x51*x62 + x53*x66;
    x69 = x54*x57;
    x70 = ddq(4) + x40;
    x71 = x69 + x70;
    x72 = x53*x71;
    x73 = ((x55)*(x55));
    x74 = ((x54)*(x54));
    x75 = -x74;
    x76 = x73 + x75;
    x77 = x51*x76 + x72;
    x78 = x52*x71 + x53*x76;
    x79 = -x69;
    x80 = x70 + x79;
    x81 = -x73;
    x82 = ((x57)*(x57));
    x83 = x81 + x82;
    x84 = x51*x80 + x53*x83;
    x85 = x52*x83 + x53*x80;
    x86 = x52*x61 + x53*x65;
    x87 = x51*x65 + x53*x61;
    x88 = x51*x71;
    x89 = -x82;
    x90 = x81 + x89;
    x91 = x53*x90;
    x92 = -0.51*x56 + 0.51*x64;
    x93 = -x43;
    x94 = 0.11*x37 + x47 + 0.51*x50 + 0.11*x93;
    x95 = -x94;
    x96 = x53*x95 + 0.11*x88 + 0.11*x91 + x92;
    x97 = x51*x90;
    x98 = x51*x95 - 0.11*x72 + 0.11*x97;
    x99 = x75 + x81;
    x100 = x51*x99;
    x101 = -x70;
    x102 = x101 + x69;
    x103 = x102*x53;
    x104 = -0.51*x62;
    x105 = 0.11*x100 + 0.11*x103 + x104 + x52*x94;
    x106 = x53*x99;
    x107 = x102*x51;
    x108 = -0.11*x106 + 0.11*x107 + x53*x94;
    x109 = cos(q(5));
    x110 = sin(q(5));
    x111 = -x55;
    x112 = x109*x54 + x110*x111;
    x113 = x109*x111 + x110*x63;
    x114 = x112*x113;
    x115 = -x114;
    x116 = dq(5)*x113 + x101*x110 + x109*x58;
    x117 = -x110;
    x118 = dq(5) + x57;
    x119 = x112*x118;
    x120 = x109*x116 + x117*x119;
    x121 = x115*x51 + x120*x53;
    x122 = x115*x53 + x120*x52;
    x123 = ((x113)*(x113));
    x124 = -x123;
    x125 = ((x112)*(x112));
    x126 = x124 + x125;
    x127 = -x119;
    x128 = -x112;
    x129 = dq(5)*x128 + x101*x109 - x110*x58;
    x130 = x127 + x129;
    x131 = x113*x118;
    x132 = x116 + x131;
    x133 = x109*x130 + x117*x132;
    x134 = x126*x51 + x133*x53;
    x135 = x126*x53 + x133*x52;
    x136 = x116 - x131;
    x137 = ((x118)*(x118));
    x138 = -x125;
    x139 = x137 + x138;
    x140 = ddq(5) + x64;
    x141 = x114 + x140;
    x142 = x109*x141;
    x143 = x117*x139 + x142;
    x144 = x136*x51 + x143*x53;
    x145 = x136*x53 + x143*x52;
    x146 = x115 + x140;
    x147 = -x137;
    x148 = x123 + x147;
    x149 = x109*x148 + x117*x146;
    x150 = x119 + x129;
    x151 = x149*x53 + x150*x51;
    x152 = x149*x52 + x150*x53;
    x153 = x109*x131 + x117*x127;
    x154 = x140*x53 + x153*x52;
    x155 = x140*x51 + x153*x53;
    x156 = x124 + x147;
    x157 = -x109;
    x158 = -0.51*x117*x156 - 0.51*x141*x157;
    x159 = -x129;
    x160 = x119 + x159;
    x161 = x160*x53;
    x162 = 0.08662*x62 + x94;
    x163 = -9.81*x1;
    x164 = 0.51*x12 + x163;
    x165 = -x16;
    x166 = 9.81*x0;
    x167 = 0.51*x165 + x166 + 0.51*x17;
    x168 = x164*x19 + x167*x28;
    x169 = x168 - 0.11*x27 + 0.11*x29 + 0.51*x41;
    x170 = x164*x20 + x167*x19;
    x171 = 0.51*x42 + 0.51*x93;
    x172 = x24 + x33;
    x173 = x170 + x171 + 0.11*x172;
    x174 = x169*x52 + x173*x53;
    x175 = 0.08662*x102 + x174;
    x176 = x117*x175 + x157*x162;
    x177 = -0.08662*x110;
    x178 = x169*x53 + x173*x51;
    x179 = x178 + 0.08662*x99;
    x180 = -x179;
    x181 = x117*x180 - 0.08662*x142 + x156*x177;
    x182 = x117*x141;
    x183 = x109*x156;
    x184 = x182 + x183;
    x185 = x184*x51;
    x186 = -0.11*x161 + x176*x51 + x181*x53 + 0.11*x185;
    x187 = x184*x53;
    x188 = x160*x51;
    x189 = x158 + x176*x53 + x181*x52 + 0.11*x187 + 0.11*x188;
    x190 = x132*x53;
    x191 = x109*x175 + x117*x162;
    x192 = -x191;
    x193 = -0.08662*x109;
    x194 = x138 + x147;
    x195 = x114 - x140;
    x196 = x109*x179 + x177*x195 + x193*x194;
    x197 = x109*x195;
    x198 = x117*x194 + x197;
    x199 = x198*x51;
    x200 = -0.11*x190 + x192*x51 + x196*x53 + 0.11*x199;
    x201 = x198*x53;
    x202 = x132*x51;
    x203 = -0.51*x117*x195 - 0.51*x157*x194;
    x204 = x192*x53 + x196*x52 + 0.11*x201 + 0.11*x202 + x203;
    x205 = sin(q(6));
    x206 = cos(q(6));
    x207 = x118*x206 + x128*x205;
    x208 = dq(6)*x207 + x116*x206 + x140*x205;
    x209 = x112*x206 + x118*x205;
    x210 = dq(6) - x113;
    x211 = x209*x210;
    x212 = x205*x208 + x206*x211;
    x213 = -x205;
    x214 = x206*x208 + x211*x213;
    x215 = x207*x209;
    x216 = -x215;
    x217 = -x216;
    x218 = x109*x214 + x117*x217;
    x219 = x212*x53 + x218*x52;
    x220 = x212*x51 + x218*x53;
    x221 = -dq(6)*x209 - x116*x205 + x140*x206;
    x222 = -x211;
    x223 = x221 + x222;
    x224 = x207*x210;
    x225 = x208 + x224;
    x226 = x205*x223 + x206*x225;
    x227 = ((x209)*(x209));
    x228 = ((x207)*(x207));
    x229 = -x228;
    x230 = x227 + x229;
    x231 = -x230;
    x232 = x206*x223 + x213*x225;
    x233 = x109*x232 + x117*x231;
    x234 = x226*x53 + x233*x52;
    x235 = x226*x51 + x233*x53;
    x236 = ddq(6) + x159;
    x237 = x215 + x236;
    x238 = x206*x237;
    x239 = -x227;
    x240 = ((x210)*(x210));
    x241 = x239 + x240;
    x242 = x213*x241 + x238;
    x243 = x208 - x224;
    x244 = -x243;
    x245 = x109*x242 + x117*x244;
    x246 = x205*x237;
    x247 = x206*x241 + x246;
    x248 = x245*x52 + x247*x53;
    x249 = x245*x53 + x247*x51;
    x250 = x216 + x236;
    x251 = -x240;
    x252 = x228 + x251;
    x253 = x205*x252 + x206*x250;
    x254 = x206*x252 + x213*x250;
    x255 = x211 + x221;
    x256 = -x255;
    x257 = x109*x254 + x117*x256;
    x258 = x253*x51 + x257*x53;
    x259 = x253*x53 + x257*x52;
    x260 = -x236;
    x261 = x206*x224 + x213*x222;
    x262 = x109*x261 + x117*x260;
    x263 = x205*x224 + x206*x222;
    x264 = x262*x53 + x263*x51;
    x265 = x262*x52 + x263*x53;
    x266 = x229 + x251;
    x267 = x206*x266;
    x268 = x213*x237 + x267;
    x269 = -x211 + x221;
    x270 = -0.035*x132 + x179;
    x271 = x191 - 0.035*x195;
    x272 = x206*x270 + x213*x271;
    x273 = -x272;
    x274 = -x176 + 0.035*x194;
    x275 = -x274;
    x276 = x205*x266;
    x277 = x213*x275 - 0.035*x238 - 0.035*x276;
    x278 = x109*x277 + x117*x273 + x177*x268 + x193*x269;
    x279 = x206*x275 - 0.035*x246 + 0.035*x267;
    x280 = x238 + x276;
    x281 = x280*x53;
    x282 = x109*x268 + x117*x269;
    x283 = x282*x51;
    x284 = x278*x53 + x279*x51 - 0.11*x281 + 0.11*x283;
    x285 = x280*x51;
    x286 = -0.51*x117*x268 - 0.51*x157*x269;
    x287 = x282*x53;
    x288 = x278*x52 + x279*x53 + 0.11*x285 + x286 + 0.11*x287;
    x289 = x215 + x260;
    x290 = x206*x289;
    x291 = x239 + x251;
    x292 = x205*x274 - 0.035*x205*x291 + 0.035*x290;
    x293 = x213*x291 + x290;
    x294 = -x225;
    x295 = -0.51*x117*x293 - 0.51*x157*x294;
    x296 = x109*x293 + x117*x294;
    x297 = x296*x53;
    x298 = x206*x291;
    x299 = x205*x289;
    x300 = x298 + x299;
    x301 = x300*x51;
    x302 = -x205*x270 - x206*x271;
    x303 = -x302;
    x304 = x206*x274 - 0.035*x298 - 0.035*x299;
    x305 = x109*x304 + x117*x303 + x177*x293 + x193*x294;
    x306 = x292*x53 + x295 + 0.11*x297 + 0.11*x301 + x305*x52;
    x307 = x300*x53;
    x308 = x296*x51;
    x309 = x292*x51 + x305*x53 - 0.11*x307 + 0.11*x308;
    x310 = x36 + x93;
    x311 = x27 - x29;
    x312 = -0.51*x18;
    x313 = -x170;
    x314 = x74 + x89;
    x315 = x58 - x61;
    x316 = x56 + x64;
    x317 = x178 + 0.51*x72 - 0.51*x97;
    x318 = -x174;
    x319 = 0.51*x106 - 0.51*x107 + x318;
    x320 = x116*x117 + x119*x157;
    x321 = x117*x130 + x132*x157;
    x322 = x139*x157 + x182;
    x323 = x117*x148 + x146*x157;
    x324 = x117*x131 + x127*x157;
    x325 = 0.08662*x110;
    x326 = x141*x325 + x157*x180 - 0.08662*x183;
    x327 = 0.51*x161 - 0.51*x185 + x326;
    x328 = x117*x179 + x194*x325 - 0.08662*x197;
    x329 = 0.51*x190 - 0.51*x199 + x328;
    x330 = x117*x214 + x157*x217;
    x331 = x117*x232 + x157*x231;
    x332 = x117*x242 + x157*x244;
    x333 = x117*x254 + x157*x256;
    x334 = x117*x261 + x157*x260;
    x335 = x117*x277 + x157*x273 + x193*x268 + x269*x325;
    x336 = 0.51*x281 - 0.51*x283 + x335;
    x337 = x117*x304 + x157*x303 + x193*x293 + x294*x325;
    x338 = 0.51*x307 - 0.51*x308 + x337;
  
    Hb(1) = ddq(1);
    Hb(2) = dq(1);
    Hb(3) = sign(dq(1));
    Hb(4) = x0*x5 + x9;
    Hb(5) = x0*(x10 + x8) + x2*(x4 + x5);
    Hb(6) = x0*x12 + x2*(-x13 + x14);
    Hb(7) = x0*(x16 + x17) + x2*(ddq(2) + x15);
    Hb(8) = x0*x4 - x9;
    Hb(9) = 0;
    Hb(10) = 0;
    Hb(11) = 0;
    Hb(12) = 0;
    Hb(13) = x0*(x19*x27 + x24*x28) + x2*(x19*x24 + x20*x27);
    Hb(14) = x0*(x19*x34 + x28*x30) + x2*(x19*x30 + x20*x34);
    Hb(15) = x0*(x19*x41 + x28*x38) + x2*(x19*x38 + x20*x41);
    Hb(16) = x0*(x19*x44 + x28*x46) + x2*(x19*x46 + x20*x44);
    Hb(17) = x0*(x19*x29 + x28*x31) + x2*(x19*x31 + x20*x29);
    Hb(18) = x0*x28*x48 + x2*(x19*x48 - 0.51*x50);
    Hb(19) = x0*x19*x47 + x2*(x20*x47 - 0.51*x30);
    Hb(20) = 0;
    Hb(21) = 0;
    Hb(22) = x0*(x19*x59 + x28*x60) + x2*(x19*x60 + x20*x59);
    Hb(23) = x0*(x19*x68 + x28*x67) + x2*(x19*x67 + x20*x68);
    Hb(24) = x0*(x19*x77 + x28*x78) + x2*(x19*x78 + x20*x77);
    Hb(25) = x0*(x19*x84 + x28*x85) + x2*(x19*x85 + x20*x84);
    Hb(26) = x0*(x19*x87 + x28*x86) + x2*(x19*x86 + x20*x87);
    Hb(27) = x0*(x19*x98 + x28*x96) + x2*(x19*x96 + x20*x98 + x92);
    Hb(28) = x0*(x105*x28 + x108*x19) + x2*(x104 + x105*x19 + x108*x20);
    Hb(29) = 0;
    Hb(30) = 0;
    Hb(31) = x0*(x121*x19 + x122*x28) + x2*(x121*x20 + x122*x19);
    Hb(32) = x0*(x134*x19 + x135*x28) + x2*(x134*x20 + x135*x19);
    Hb(33) = x0*(x144*x19 + x145*x28) + x2*(x144*x20 + x145*x19);
    Hb(34) = x0*(x151*x19 + x152*x28) + x2*(x151*x20 + x152*x19);
    Hb(35) = x0*(x154*x28 + x155*x19) + x2*(x154*x19 + x155*x20);
    Hb(36) = x0*(x186*x19 + x189*x28) + x2*(x158 + x186*x20 + x189*x19);
    Hb(37) = x0*(x19*x200 + x204*x28) + x2*(x19*x204 + x20*x200 + x203);
    Hb(38) = 0;
    Hb(39) = 0;
    Hb(40) = x0*(x19*x220 + x219*x28) + x2*(x19*x219 + x20*x220);
    Hb(41) = x0*(x19*x235 + x234*x28) + x2*(x19*x234 + x20*x235);
    Hb(42) = x0*(x19*x249 + x248*x28) + x2*(x19*x248 + x20*x249);
    Hb(43) = x0*(x19*x258 + x259*x28) + x2*(x19*x259 + x20*x258);
    Hb(44) = x0*(x19*x264 + x265*x28) + x2*(x19*x265 + x20*x264);
    Hb(45) = x0*(x19*x284 + x28*x288) + x2*(x19*x288 + x20*x284 + x286);
    Hb(46) = x0*(x19*x309 + x28*x306) + x2*(x19*x306 + x20*x309 + x295);
    Hb(47) = 0;
    Hb(48) = 0;
    Hb(49) = 0;
    Hb(50) = 0;
    Hb(51) = 0;
    Hb(52) = x15;
    Hb(53) = x13 + x165;
    Hb(54) = -x4 + x5;
    Hb(55) = x10 + x7;
    Hb(56) = ddq(2);
    Hb(57) = x163;
    Hb(58) = -x166;
    Hb(59) = dq(2);
    Hb(60) = sign(dq(2));
    Hb(61) = x45;
    Hb(62) = x310;
    Hb(63) = x311;
    Hb(64) = x172;
    Hb(65) = x40;
    Hb(66) = x168 + x171*x20 + x312*x41;
    Hb(67) = 0.51*x20*(x39 - x40) + x312*(x37 + x42) + x313;
    Hb(68) = 0;
    Hb(69) = 0;
    Hb(70) = x79;
    Hb(71) = x314;
    Hb(72) = x315;
    Hb(73) = x316;
    Hb(74) = x70;
    Hb(75) = 0.51*x20*(x88 + x91) + x312*(x52*x90 + x72) + x317;
    Hb(76) = 0.51*x20*(x100 + x103) + x312*(x102*x52 + x106) + x319;
    Hb(77) = 0;
    Hb(78) = 0;
    Hb(79) = x320;
    Hb(80) = x321;
    Hb(81) = x322;
    Hb(82) = x323;
    Hb(83) = x324;
    Hb(84) = 0.51*x20*(x187 + x188) + x312*(x161 + x184*x52) + x327;
    Hb(85) = 0.51*x20*(x201 + x202) + x312*(x190 + x198*x52) + x329;
    Hb(86) = 0;
    Hb(87) = 0;
    Hb(88) = x330;
    Hb(89) = x331;
    Hb(90) = x332;
    Hb(91) = x333;
    Hb(92) = x334;
    Hb(93) = 0.51*x20*(x285 + x287) + x312*(x281 + x282*x52) + x336;
    Hb(94) = 0.51*x20*(x297 + x301) + x312*(x296*x52 + x307) + x338;
    Hb(95) = 0;
    Hb(96) = 0;
    Hb(97) = 0;
    Hb(98) = 0;
    Hb(99) = 0;
    Hb(100) = 0;
    Hb(101) = 0;
    Hb(102) = 0;
    Hb(103) = 0;
    Hb(104) = 0;
    Hb(105) = 0;
    Hb(106) = 0;
    Hb(107) = 0;
    Hb(108) = 0;
    Hb(109) = x45;
    Hb(110) = x310;
    Hb(111) = x311;
    Hb(112) = x172;
    Hb(113) = x40;
    Hb(114) = x168;
    Hb(115) = x313;
    Hb(116) = dq(3);
    Hb(117) = sign(dq(3));
    Hb(118) = x79;
    Hb(119) = x314;
    Hb(120) = x315;
    Hb(121) = x316;
    Hb(122) = x70;
    Hb(123) = x317;
    Hb(124) = x319;
    Hb(125) = 0;
    Hb(126) = 0;
    Hb(127) = x320;
    Hb(128) = x321;
    Hb(129) = x322;
    Hb(130) = x323;
    Hb(131) = x324;
    Hb(132) = x327;
    Hb(133) = x329;
    Hb(134) = 0;
    Hb(135) = 0;
    Hb(136) = x330;
    Hb(137) = x331;
    Hb(138) = x332;
    Hb(139) = x333;
    Hb(140) = x334;
    Hb(141) = x336;
    Hb(142) = x338;
    Hb(143) = 0;
    Hb(144) = 0;
    Hb(145) = 0;
    Hb(146) = 0;
    Hb(147) = 0;
    Hb(148) = 0;
    Hb(149) = 0;
    Hb(150) = 0;
    Hb(151) = 0;
    Hb(152) = 0;
    Hb(153) = 0;
    Hb(154) = 0;
    Hb(155) = 0;
    Hb(156) = 0;
    Hb(157) = 0;
    Hb(158) = 0;
    Hb(159) = 0;
    Hb(160) = 0;
    Hb(161) = 0;
    Hb(162) = 0;
    Hb(163) = 0;
    Hb(164) = 0;
    Hb(165) = 0;
    Hb(166) = x79;
    Hb(167) = x314;
    Hb(168) = x315;
    Hb(169) = x316;
    Hb(170) = x70;
    Hb(171) = x178;
    Hb(172) = x318;
    Hb(173) = dq(4);
    Hb(174) = sign(dq(4));
    Hb(175) = x320;
    Hb(176) = x321;
    Hb(177) = x322;
    Hb(178) = x323;
    Hb(179) = x324;
    Hb(180) = x326;
    Hb(181) = x328;
    Hb(182) = 0;
    Hb(183) = 0;
    Hb(184) = x330;
    Hb(185) = x331;
    Hb(186) = x332;
    Hb(187) = x333;
    Hb(188) = x334;
    Hb(189) = x335;
    Hb(190) = x337;
    Hb(191) = 0;
    Hb(192) = 0;
    Hb(193) = 0;
    Hb(194) = 0;
    Hb(195) = 0;
    Hb(196) = 0;
    Hb(197) = 0;
    Hb(198) = 0;
    Hb(199) = 0;
    Hb(200) = 0;
    Hb(201) = 0;
    Hb(202) = 0;
    Hb(203) = 0;
    Hb(204) = 0;
    Hb(205) = 0;
    Hb(206) = 0;
    Hb(207) = 0;
    Hb(208) = 0;
    Hb(209) = 0;
    Hb(210) = 0;
    Hb(211) = 0;
    Hb(212) = 0;
    Hb(213) = 0;
    Hb(214) = 0;
    Hb(215) = 0;
    Hb(216) = 0;
    Hb(217) = 0;
    Hb(218) = 0;
    Hb(219) = 0;
    Hb(220) = 0;
    Hb(221) = 0;
    Hb(222) = 0;
    Hb(223) = x115;
    Hb(224) = x126;
    Hb(225) = x136;
    Hb(226) = x150;
    Hb(227) = x140;
    Hb(228) = x176;
    Hb(229) = x192;
    Hb(230) = dq(5);
    Hb(231) = sign(dq(5));
    Hb(232) = x212;
    Hb(233) = x226;
    Hb(234) = x247;
    Hb(235) = x253;
    Hb(236) = x263;
    Hb(237) = x279;
    Hb(238) = x292;
    Hb(239) = 0;
    Hb(240) = 0;
    Hb(241) = 0;
    Hb(242) = 0;
    Hb(243) = 0;
    Hb(244) = 0;
    Hb(245) = 0;
    Hb(246) = 0;
    Hb(247) = 0;
    Hb(248) = 0;
    Hb(249) = 0;
    Hb(250) = 0;
    Hb(251) = 0;
    Hb(252) = 0;
    Hb(253) = 0;
    Hb(254) = 0;
    Hb(255) = 0;
    Hb(256) = 0;
    Hb(257) = 0;
    Hb(258) = 0;
    Hb(259) = 0;
    Hb(260) = 0;
    Hb(261) = 0;
    Hb(262) = 0;
    Hb(263) = 0;
    Hb(264) = 0;
    Hb(265) = 0;
    Hb(266) = 0;
    Hb(267) = 0;
    Hb(268) = 0;
    Hb(269) = 0;
    Hb(270) = 0;
    Hb(271) = 0;
    Hb(272) = 0;
    Hb(273) = 0;
    Hb(274) = 0;
    Hb(275) = 0;
    Hb(276) = 0;
    Hb(277) = 0;
    Hb(278) = 0;
    Hb(279) = 0;
    Hb(280) = x216;
    Hb(281) = x230;
    Hb(282) = x243;
    Hb(283) = x255;
    Hb(284) = x236;
    Hb(285) = x272;
    Hb(286) = x302;
    Hb(287) = dq(6);
    Hb(288) = sign(dq(6));
    tmp_mat = reshape(Hb,[],6);
    Hb = tmp_mat';
end