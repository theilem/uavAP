function coefs25 = regression25(extra_constants)
    coefs25 = struct();

    if nargin<1 || isempty(extra_constants)
        coefs25.X0	=	-0.03234971795	;
        coefs25.Z0	=	-0.1845993499	;
        coefs25.M0	=	-0.003408601817	;
        coefs25.Y0	=	0.06372415162	;
        coefs25.L0	=	0.4900128479	;
        coefs25.N0	=	-0.01130310995	;
    else
        coefs25.X0 = extra_constants.u_dot;
        coefs25.Z0 = extra_constants.w_dot;
        coefs25.M0 = extra_constants.q_dot;
        coefs25.Y0 = extra_constants.v_dot;
        coefs25.L0 = extra_constants.p_dot;
        coefs25.N0 = extra_constants.r_dot;
    end

    coefs25.Xu	=	-0.2487694716	;
    coefs25.Xw	=	0.6993053813	;
    coefs25.Xq	=	0	;
    coefs25.Xv	=	0	;
    coefs25.Xp	=	0	;
    coefs25.Xr	=	0	;
    coefs25.Xct	=	3.301037339	;
    coefs25.Xce	=	0	;
    coefs25.Xca	=	0	;
    coefs25.Xcr	=	0	;

    coefs25.Zu	=	-0.4670556652	;
    coefs25.Zw	=	-10.60108977	;
    coefs25.Zq	=	20.22795564	;
    coefs25.Zv	=	0	;
    coefs25.Zp	=	0	;
    coefs25.Zr	=	0	;
    coefs25.Zct	=	0	;
    coefs25.Zce	=	10.22902361	;
    coefs25.Zca	=	0	;
    coefs25.Zcr	=	0	;

    coefs25.Mu	=	0	;
    coefs25.Mw	=	-17.99911557	;
    coefs25.Mq	=	-37.22580371	;
    coefs25.Mv	=	0	;
    coefs25.Mp	=	0	;
    coefs25.Mr	=	0	;
    coefs25.Mct	=	0.05908759573	;
    coefs25.Mce	=	75.11968436	;
    coefs25.Mca	=	0	;
    coefs25.Mcr	=	0	;

    coefs25.Yu	=	0	;
    coefs25.Yw	=	0	;
    coefs25.Yq	=	0	;
    coefs25.Yv	=	-1.522928148	;
    coefs25.Yp	=	0.7743133806	;
    coefs25.Yr	=	-25.11097309	;
    coefs25.Yct	=	0	;
    coefs25.Yce	=	0	;
    coefs25.Yca	=	0	;
    coefs25.Ycr	=	-2.331856512	;

    coefs25.Lu	=	0	;
    coefs25.Lw	=	0	;
    coefs25.Lq	=	0	;
    coefs25.Lv	=	0	;
    coefs25.Lp	=	-38.26706286	;
    coefs25.Lr	=	0	;
    coefs25.Lct	=	0	;
    coefs25.Lce	=	0	;
    coefs25.Lca	=	150.2219674	;
    coefs25.Lcr	=	0	;

    coefs25.Nu	=	0	;
    coefs25.Nw	=	0	;
    coefs25.Nq	=	0	;
    coefs25.Nv	=	3.375952988	;
    coefs25.Np	=	-0.2430081261	;
    coefs25.Nr	=	-3.379496955	;
    coefs25.Nct	=	0	;
    coefs25.Nce	=	0	;
    coefs25.Nca	=	-1.248455012	;
    coefs25.Ncr	=	12.59622001	;
end