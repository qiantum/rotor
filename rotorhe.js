// Copyright: Qianyan Cai
// License: GPL v3

// 双旋轮线转子引擎 Hypo-Epitrochoid Rotorary Engine
function RotorHE({
	N, // 倍数、工作区数
	E, // 偏心距
	Q = 0.8, // 缸体拱半径 / 偏心距
	RB = 0.3, // 转子缸体间隙Z
	GE = 0, // 节圆变位加粗曲轴
	tickn = 240, // 圆周步进数
	size, // 预估像素
	evn,
}) {
	if (N != (N |= 0) || N < 2) throw 'err N'
	let N1 = N - 1
	let NB = N // 缸体顶数
	let NR = N1 // 转子顶数
	let NE = -NR // 曲轴速比
	let NS = N1 + N1 // 圆周冲程数
	let NS4 = lcm(NS, 4) // 完整循环冲程数
	tickn = ceil(tickn / NB / NR / 2) * NB * NR * 2 // 圆周步进数，缸体顶*转子顶*2 的整倍数

	size = ceil(+size || min(size.width, size.height))
	E ??= evn ? (size * 0.1334) / sqrt(Q + N + 2.9) : (size * 0.108) / sqrt(Q + N + 2.8)
	Q = round(E * (Q + N + 4) * 4) / 4 // 缸体拱半径
	E = round(E * 3) / 3 // 偏心距
	if ((E | 0) < 1) throw 'err E'
	let GB = E * NB // 缸体节圆半径
	let G = E * NR // 转子节圆半径
	let P = Q - E - E // 缸体顶半径
	let RP = Q - E - RB // 转子顶半径

	// 缸体、曲轴步进角，均匀
	let Tick_ = (this.Tick_ = sequ(0, tickn).map(t => (t / tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, tickn))
	let tS = S => ((S % NS) * tickn) / NS // 冲程起始步进
	let TS = S => (S * PI2) / NS // 冲程起始角
	// 缸体顶：
	let tPQ = tickn / 2 / N // 半边步进
	let TPQ = PI / N // 半边夹角
	let tN = n => (n * tickn) / N // 起始步进
	let TN = n => (n * PI2) / N // 起始角
	// 转子顶：
	let tPQR = tickn / 2 / NR // 顶腰步进
	let TPQR = PI / NR // 顶腰夹角
	let tNR = n => (n * tickn) / NR // 转子起始步进
	let TNR = n => (n * PI2) / NR // 转子起始角

	// 曲轴心 X=0 Y=0B
	let GX = T => E * cos(T * NE) // 转子心X
	let GY = T => E * sin(T * NE) // 转子心Y
	let PX = (n, T, p = P) => p * cos(TN(n) + TPQ) // 缸体顶X
	let PY = (n, T, p = P) => p * sin(TN(n) + TPQ) // 缸体顶Y

	// 转子点XY，R顶腰处Tick整倍数，顶X==GX(T)+RP*cos(T+TNR(n)) 顶Y==GY(T)+RP*sin(T+TNR(n))
	let RX = (R, T, rp = RP) => (rp - E) * cos(R + T) + E * cos(R * NB + T) + GX(T)
	let RY = (R, T, rp = RP) => (rp - E) * sin(R + T) + E * sin(R * NB + T) + GY(T)
	// 转子型线、即顶型线、即缸体顶绕转子心的外旋轮线
	function* RR(T, Rt, RT = Tick_) {
		for (let R of Rt?.imap(Tick_.At) ?? RT) yield [RX(R, T), RY(R, T)]
	}

	let TSt = Tick_.map(R => atan(RX(R, 0) - GX(0), RY(R, 0) - GY(0)))
	TSt[TSt.length - 1] = PI2 // 顶型线步进角，非均匀
	// 转子工作线步进，每TS(s)T为tickn整倍数，短于转子边，缸体工作线==缸体边==St(0,n)
	function St(T, n = 0, rev) {
		if (abs(T) < EPSI && n == 0) return sequ(-tPQ, tPQ, tickn, false, rev) // ==St(0,0)
		let A1 = (atan(PX(n - 1) - GX(T), PY(n - 1) - GY(T)) - T).mod(PI2)
		let A = (atan(PX(n) - GX(T), PY(n) - GY(T)) - T).mod(PI2)
		return sequ(round(A1.bfind(TSt)) % tickn, round(A.bfind(TSt)) % tickn, tickn, false, rev)
	}

	// 顶型线旋转 A:顶步进角 T:旋转步进角
	function* PAT(A, p) {
		if (typeof A == 'number')
			for (let T of Tick_) {
				let X = RX(A, T, p) // 转子对缸体心旋转
				let Y = RY(A, T, p)
				yield atandist(X, Y) // 角[0,PI2) A==0 沿T严格递增 A>0 沿T循环严格递增
			}
		else for (A of A) yield PAT(A, p)
	}
	let RPT = sequ(-tPQR, tPQR, tickn, true).map(Tick.At) // 转子边步进角
	// 缸体型线、即转子绕曲轴的外包络线
	let BB = [...enve(max, PAT(RPT, RP + RB), t => ((t / tPQ) % 2 == 1 ? P : null))()]
	// 腰包络、即转子绕曲轴的内包络线
	let QQ = enve(min, PAT(RPT, RP))

	// 工作区型线
	let SS = (T, n = 0, _ = St(0, n).imap(BB.At)) => _.iconcat(RR(T, St(T, n, true))).iclose()
	let V0 = area(SS(0)) // 最小容积
	let VS = (T, n = 0, add0) => area(SS(T, n)) - (add0 ? 0 : V0) // 工作区容积
	let V = VS(TS(1)) // 工作容积
	let K = V / V0 + 1 // 容积比，即压缩比、膨胀比
	let VN = V * N // 循环排量
	let VB = V * NB // 几何排量
	let VV = area(BB) // 总体积
	let KK = VV / V // 总体积比工作容积

	let QRT = T => QQ(T, St(0), -E * cos(T * N), -E * sin(T * N)) // 腰包络绕转子心
	// 冲程区型线
	let SSS = sequ(0, NS - 1).map(S => {
		let dots = sequ(tS(-S - 1), tS(-S), tickn, true).imap(t =>
			QRT(Tick_[t]).imap(([X, Y]) => atandist(X, Y))
		)
		let st = [...sequ(tS(-S - 1) - tPQ, tS(-S) + tPQ, tickn, true)] // 转子工作线，此处tickn整倍数
		let strev = st.rev()
		let pp = enve(min, dots, null, TSt, st, 1)
		return T => RR(T, st).iconcat(pp(T, strev)).iclose()
	})

	// 缸体顶与转子接触角、及接触步进角
	function RBC(T, n = 0) {
		let CT = atan(PX(n) - GB * cos(T * NE), PY(n) - GB * sin(T * NE)) // 两节圆交点--缸体顶点
		return [diffabs(TN(n) + TPQ - CT), CT]
	}
	let RBCC = max(...Tick.map(T => RBC(T)[0])) // 最大接触角

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { N, NR, NS: NS4, E, GB, G, GE, P, Q, RB, V, K, VN, VB, VV, KK, RBCC })
	Object.assign(this, { size, GX, GY, TN, TS, BB, RR, SS, VS })

	// 参数显示
	function params(T) {
		let p = ''
		if (T != null) {
			p = _`|${(RBC(T)[0] / PI2) * 360}{02}__`
			let a = (T / TS(1)) * PI
			let pis = (3 + 1 - sqrt(3 * 3 - sin(a) * sin(a)) - cos(a)) / 2
			p += _`${VS(T) / 100}{03}:${VS(T) / V}{2}|${(1 - cos(a)) / 2}{2}|${pis}{2}`
		}
		return (
			_`N${N}__E${E}{}__Q${Q}{}__K${K}{.1}__` +
			_`V${V / 100}{} ${VV / 100}{}__${VB / 100}{} ${VN / 100}{}__` +
			_`RB${RB}{2} C${(RBCC / PI2) * 360}{}` +
			p
		)
	}
	console.log(...params().split('__'), _`Vmin${V0 / 100}{.1} tn${tickn}`)

	this.$ = ({ canvas, midx = 0, midy = 0, x, y, param }) => {
		x ??= canvas.width / 2 + midx // 曲轴心X
		y ??= canvas.height / 2 + midy // 曲轴心Y
		let $param = T => (param.textContent = params(T).replace(/__/g, '\n'))
		let $ = canvas.getContext('2d')
		let gb = GB + E * GE
		let g = G + E * GE
		let gbw = (gb * PI) / max(NB * 4, 9 - 3 * (NR - NB))
		let gw = (g * PI) / max(NR * 4, 9 + 3 * (NR - NB))
		let gwi = abs(gbw - gw) / 2

		function $$({ color = '#000', opa = '', thick = 1, dash } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			opa.length > 0 && /#....(....)?$/.test(color) && (color = color.slice(0, -opa.length))
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
			dash && $.setLineDash(dash)
		}
		function $$$(fill) {
			fill ? $.fill() : $.stroke(), ($.lineWidth = 1)
			fill ? ($.fillStyle = '#000') : ($.strokeStyle = '#000')
			$.setLineDash([])
		}
		// 画偏心线，即曲轴
		function $E(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo(x, y), $.lineTo(x + GX(T), y - GY(T)), $$$()
		}
		// 画缸体节圆
		function $GB(style) {
			$$({ color: '#333', dash: [0, gbw, gbw - gwi, gwi], ...style })
			$.arc(x, y, gb, 0, PI2), $$$()
		}
		// 画转子节圆
		function $G(T, style) {
			$$({ color: '#999', dash: [gw - gwi, gw + gwi], ...style })
			$.arc(x + GX(T), y - GY(T), g, -T, PI2 - T), $$$()
		}
		// 画缸体节圆绕转子外包
		function $GG(T, style) {
			$$({ color: '#ccc', ...style }), $.arc(x + GX(T), y - GY(T), gb + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $RP(T, nr = 0, O, style) {
			$$({ color: '#00f', ...style })
			let R = TNR(nr)
			$.moveTo(x + RX(R, T), y - RY(R, T)), $.lineTo(x + RX(R, T, O), y - RY(R, T, O))
			$$$()
		}
		// 画转子腰
		function $RQ(T, nr = 0, O, style) {
			$$({ color: '#f33', ...style })
			let R = TNR(nr) + TPQR
			$.moveTo(x + RX(R, T), y - RY(R, T)), $.lineTo(x + RX(R, T, O), y - RY(R, T, O))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = [0, G], style) {
			for (let nr = 0; nr < NR; nr++) $RP(T, nr, O?.[nr] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = G, style) {
			for (let nr = 0; nr < NR; nr++) $RQ(T, nr, O?.[nr] ?? O?.at?.(-1) ?? O, style)
		}
		// 画间隙密封
		function $RB(T, style) {
			if (!RB) return
			for (let Style = { color: '#f00', ...style }, n = 0; n < N; n++)
				$$(Style), $.arc(x + PX(n), y - PY(n), RB, 0, PI2), $$$()
		}
		// 画腰包络
		function $QQ(style) {
			$$({ color: '#fbb', ...style })
			let to
			for (let [X, Y] of QQ()) (to = to ? $.lineTo : $.moveTo).call($, x + X, y - Y)
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, x + X, y - Y)
			$$$()
		}
		// 画转子
		function $RR(T, ST, style) {
			$$(style)
			let to
			for (let [X, Y] of RR(T, ST != null ? St(ST) : undefined))
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y - Y)
			$$$()
		}
		// 画工作区
		function $SS(T, n = 0, style) {
			$$(style, true)
			let to
			for (let [X, Y] of SS(T, n)) (to = to ? $.lineTo : $.moveTo).call($, x + X, y - Y)
			$$$(true)
		}
		// 画冲程区
		function $SSS(T, S, style, fill = true) {
			$$(style, fill)
			let to
			for (let [X, Y] of SSS[floor(S).mod(NS)](T))
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y - Y)
			$$$(fill)
		}
		// 画接触角
		function $RBC(T, n = 0, O = P * 0.9 - RB, style) {
			$$({ color: '#999', ...style })
			let CT = RBC(T, n)[1]
			$.moveTo(x + PX(n, T, O), y - PY(n, T, O))
			$.lineTo(x + PX(n, T), y - PY(n, T))
			$.lineTo(x + PX(n, T) + (O - P) * cos(CT), y - PY(n, T) - (O - P) * sin(CT))
			$$$()
		}
		return Object.assign(
			{ param: $param, x, y, E: $E, GB: $GB, G: $G, GG: $GG },
			{ RP: $RP, RQ: $RQ, PN: $PN, QN: $QN, RB: $RB, QQ: $QQ },
			{ BB: $BB, RR: $RR, SS: $SS, SSS: $SSS, RBC: $RBC }
		)
	}

	// 点集求包络线 dots:[[ [A, R] ]] tt:正向步进、可卷
	function enve(most = min, dots, fix, TT = Tick_, tt = Tick_.keys(), xyk = 0) {
		if (TT[0] != 0 || TT[tickn] != PI2) throw 'err TT'
		let init = most == min ? size * 10 : 0
		let M = new Array(tickn).fill(init)
		for (let dot of dots)
			for (let [A, R] of dot) {
				let t = ceil(TT == Tick_ ? (tickn * A) / PI2 : A.bfind(TT)) % tickn
				M[t] = most(M[t], R)
			}
		tt.values ?? (tt = [...tt])
		// M.fillHole(init) 如果tickn太小，部分步进无数据，则需要线性填充
		M[tt[0]] == init && (M[tt[0]] = M[(tt[0] + 1) % tickn])
		M[(tt.at(-1) + 1) % tickn] == init && (M[(tt.at(-1) + 1) % tickn] = M[tt.at(-1)])
		M.close(tickn, 0)
		for (let t of tt)
			if (t < tickn) M[t] = max(M[t], M[t + 1]) * 0.3125 + min(M[t], M[t + 1]) * 0.6875
		if (fix) for (let t of tt) M[t] = fix(t, TT[t]) ?? M[t]
		M.close(tickn, 0)
		return function* (T = 0, t_ = tt, x = xyk * GX(T), y = xyk * GY(T)) {
			for (let t of t_) yield [x + M[t] * cos(T + TT[t]), y + M[t] * sin(T + TT[t])]
		}
	}
}
