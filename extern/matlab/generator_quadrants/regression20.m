function coefs20 = regression20(extra_constants)
    coefs20 = struct();

    if nargin<1 || isempty(extra_constants)
        coefs20.X0	=	-0.07302515229	;
        coefs20.Z0	=	-0.04941860757	;
        coefs20.M0	=	-0.02798689117	;
        coefs20.Y0	=	-0.08225395555	;
        coefs20.L0	=	0.4368770043	;
        coefs20.N0	=	-0.02227796081	;
    else
        coefs20.X0 = extra_constants.u_dot;
        coefs20.Z0 = extra_constants.w_dot;
        coefs20.M0 = extra_constants.q_dot;
        coefs20.Y0 = extra_constants.v_dot;
        coefs20.L0 = extra_constants.p_dot;
        coefs20.N0 = extra_constants.r_dot;
    end

    coefs20.Xu	=	-0.2627559489	;
    coefs20.Xw	=	0.7636313034	;
    coefs20.Xq	=	-0.3503224288	;
    coefs20.Xv	=	0	;
    coefs20.Xp	=	0	;
    coefs20.Xr	=	0	;
    coefs20.Xct	=	3.493111091	;
    coefs20.Xce	=	0	;
    coefs20.Xca	=	0	;
    coefs20.Xcr	=	0	;

    coefs20.Zu	=	-0.6128803927	;
    coefs20.Zw	=	-8.144346001	;
    coefs20.Zq	=	17.50574527	;
    coefs20.Zv	=	0	;
    coefs20.Zp	=	0	;
    coefs20.Zr	=	0	;
    coefs20.Zct	=	0	;
    coefs20.Zce	=	5.221055423	;
    coefs20.Zca	=	0	;
    coefs20.Zcr	=	0	;

    coefs20.Mu	=	0.7864886217	;
    coefs20.Mw	=	-13.86457528	;
    coefs20.Mq	=	-24.96965758	;
    coefs20.Mv	=	0	;
    coefs20.Mp	=	0	;
    coefs20.Mr	=	0	;
    coefs20.Mct	=	0.1145797833	;
    coefs20.Mce	=	46.16773474	;
    coefs20.Mca	=	0	;
    coefs20.Mcr	=	0	;

    coefs20.Yu	=	0	;
    coefs20.Yw	=	0	;
    coefs20.Yq	=	0	;
    coefs20.Yv	=	-1.246266327	;
    coefs20.Yp	=	1.123655383	;
    coefs20.Yr	=	-20.06950683	;
    coefs20.Yct	=	0	;
    coefs20.Yce	=	0	;
    coefs20.Yca	=	0	;
    coefs20.Ycr	=	-1.55684152	;

    coefs20.Lu	=	0	;
    coefs20.Lw	=	0	;
    coefs20.Lq	=	0	;
    coefs20.Lv	=	0	;
    coefs20.Lp	=	-32.19717634	;
    coefs20.Lr	=	0	;
    coefs20.Lct	=	0	;
    coefs20.Lce	=	0	;
    coefs20.Lca	=	104.5968301	;
    coefs20.Lcr	=	0	;

    coefs20.Nu	=	0	;
    coefs20.Nw	=	0	;
    coefs20.Nq	=	0.2933180683	;
    coefs20.Nv	=	2.499595746	;
    coefs20.Np	=	-0.5105706621	;
    coefs20.Nr	=	-2.567884351	;
    coefs20.Nct	=	-0.03034984249	;
    coefs20.Nce	=	0	;
    coefs20.Nca	=	-0.5149368263	;
    coefs20.Ncr	=	8.370640962	;
end