// Copyright: Qianyan Cai
// License: GPL v3

// 外旋轮线转子引擎 Epitrochoid Rotorary Engine
function RotorE({
	N, // 倍数、工作区数
	E, // 偏心距
	P = N == 2 ? 1.3 : 2, // 转子顶半径 / 偏心距
	RB = 0.3, // 转子缸体间隙
	EG = N == 2 ? 0.27 : 0, // 节圆变位加粗曲轴
	tickn = 240, // 圆周步进数
	size, // 预估像素
	evn,
}) {
	if (N != (N |= 0) || N < 2) throw 'err N'
	let N1 = N - 1
	let NB = N1 // 缸体顶数
	let NR = N // 转子顶数
	let NE = NR // 曲轴速比
	let NS = N1 + N1 // 圆周冲程数
	let NS4 = lcm(NS, 4) // 完整循环冲程数
	tickn = ceil(tickn / NB / NR / 2) * NB * NR * 2 // 圆周步进数，缸体顶*转子顶*2 的整倍数

	size = ceil(+size || min(size.width, size.height))
	E ??= evn ? (size * 0.13) / sqrt(P + N + 2.75) : (size * 0.119) / sqrt(P + N * 2 - 1.5)
	P = round(E * (P + N + 2) * 2) / 2 // 转子顶半径
	E = round(E * 3) / 3 // 偏心距
	if ((E | 0) < 1) throw 'err E'
	let BG = E * NB // 缸体节圆半径
	let G = E * NR // 转子节圆半径
	let Q = P - E - E // 转子腰半径

	// 转子、曲轴步进角，均匀
	let Tick_ = (this.Tick_ = sequ(0, tickn).map(t => (t / tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, tickn))
	let tS = S => ((S % NS) * tickn) / NS // 冲程起始步进
	let TS = S => (S * PI2) / NS // 冲程起始角
	// 转子腰：
	let tPQ = tickn / 2 / N // 半边步进
	let TPQ = PI / N // 半边夹角
	let tN = n => (n * tickn) / N // 起始步进
	let TN = n => (n * PI2) / N // 起始角

	// 曲轴心 X=0 Y=0
	let GX = T => E * cos(T * NE) // 转子心X
	let GY = T => E * sin(T * NE) // 转子心Y
	let PX = (n, T, p = P) => GX(T) + p * cos(T + TN(n) + TPQ) // 转子顶X
	let PY = (n, T, p = P) => GY(T) + p * sin(T + TN(n) + TPQ) // 转子顶Y
	let QX = (n, T, q = Q) => GX(T) + q * cos(T + TN(n)) // 转子腰X
	let QY = (n, T, q = Q) => GY(T) + q * sin(T + TN(n)) // 转子腰X

	// 缸体点XY，T==0时 X==PX(0,B-TPQ,p) Y==PY(0,B-TPQ,p)
	let BX = (B, T, p = P + RB) => p * cos(B + T) - E * cos(B * NR + T)
	let BY = (B, T, p = P + RB) => p * sin(B + T) - E * sin(B * NR + T)
	// 缸体型线、即顶型线、即转子顶绕曲轴心的外旋轮线
	let BB = Tick_.map(B => [BX(B, 0), BY(B, 0)])

	let TSt = BB.map(([X, Y]) => atan(X, Y))
	TSt[TSt.length - 1] = PI2 // 顶型线步进角，非均匀
	// 缸体工作线步进，每TS(s)T为tickn整倍数，短于缸体边，转子工作线==转子边==St(0,n)
	function St(T, n = 0, rev) {
		if (abs(T) < EPSI && n == 0) return sequ(-tPQ, tPQ, tickn, false, rev) // ==St(0,0)
		let A1 = atan(PX(n - 1, T, P + RB), PY(n - 1, T, P + RB))
		let A = atan(PX(n, T, P + RB), PY(n, T, P + RB))
		return sequ(round(A1.bfind(TSt)) % tickn, round(A.bfind(TSt)) % tickn, tickn, false, rev)
	}

	// 顶型线旋转 A:顶步进角 T:旋转步进角
	function* PAT(A, p) {
		if (typeof A == 'number')
			for (let T of Tick_) {
				let X = BX(A, T, p) - E * cos(T - T * NE) // 缸体对转子心旋转
				let Y = BY(A, T, p) - E * sin(T - T * NE)
				yield atandist(X, Y) // 角[0,PI2) A==0 沿T严格递增 A>0 沿T循环严格递增
			}
		else for (A of A) yield PAT(A, p)
	}
	// 转子型线、即缸体绕转子心的内包络线 // RR = MinCurve(PAT(Tick), true)
	let RR = enve(min, PAT(St(0).imap(Tick.At), P), t => (t % tPQ ? null : t % (tickn / NR) ? P : Q))

	// 工作区型线，== ...RR(T + TN(n), St(0, 0, true))
	let SS = (T, n = 0, _ = St(T, n).imap(BB.At)) => _.iconcat(RR(T, St(0, n, true))).iclose()
	let V0 = abs(area(SS(0))) // 最小容积
	let VS = (T, n = 0, add0) => area(SS(T, n)) - (add0 ? 0 : V0) // 工作区容积
	let V = VS(TS(1)) // 工作容积
	let K = V / V0 + 1 // 容积比，即压缩比、膨胀比
	let VN = V * N // 循环排量
	let VB = V * NB // 几何排量
	let VV = area(BB) // 总体积
	let KK = VV / V // 总体积比工作容积

	// 冲程区型线
	let SSS = sequ(0, NS - 1).map(S => {
		let dots = sequ(tS(S), tS(S + 1), tickn, true).imap(t =>
			RR(Tick_[t], St(0)).imap(([X, Y]) => atandist(X, Y))
		)
		let st = [...sequ(tS(S) - tPQ, tS(S + 1) + tPQ, tickn, true)] // 缸体工作线，此处tickn整倍数
		let s = st.imap(BB.At).concat(enve(min, dots, null, TSt, st, 0)().rev()).close()
		return _ => s
	})

	// 转子顶与缸体接触角、及接触步进角
	function RBC(T, n = 0) {
		let CT = atan(PX(n, T) + BG * cos(T * NE), PY(n, T) + BG * sin(T * NE)) // 两节圆交点--转子顶点
		return [diffabs(T + TN(n) + TPQ - CT), CT]
	}
	let RBCC = max(...Tick.map(T => RBC(T)[0])) // 最大接触角

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { N, NR, NS: NS4, E, EG, BG, G, P, Q, RB, V, K, VN, VB, VV, KK, RBCC })
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
			_`N${N}__E${E}{}__P${P}{}__K${K}{.1}__` +
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
		$.Arc = (x, y, r, a, b) => $.arc(x, y, r, -a, -b, true)
		let bg = BG + E * EG
		let g = G + E * EG
		let bgz = (bg * PI) / max(NB * 4, 9 - 3 * (NR - NB))
		let gz = (g * PI) / max(NR * 4, 9 + 3 * (NR - NB))
		let gzi = abs(bgz - gz) / 2

		function $$({ color = '#000', opa = '', thick = 1, dash } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			opa.length > 0 && /#....(....)?$/.test(color) && (color = color.slice(0, -opa.length))
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
			dash && $.setLineDash(dash)
		}
		function $$$(fill, close) {
			close && $.closePath()
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
		function $BG(T, style) {
			$$({ color: '#333', dash: [0, bgz, bgz - gzi, gzi], ...style })
			$.Arc(x, y, bg, 0, PI2), $$$()
		}
		// 画转子节圆
		function $G(T, style) {
			$$({ color: '#999', dash: [gz - gzi, gz + gzi], ...style })
			$.Arc(x + GX(T), y - GY(T), g, T, PI2 + T), $$$()
		}
		// 画转子节圆外包
		function $GG(T, style) {
			$$({ color: '#ccc', ...style }), $.Arc(x, y, g + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $RP(T, nr = 0, O, style) {
			$$({ color: '#00f', ...style })
			$.moveTo(x + PX(nr, T), y - PY(nr, T)), $.lineTo(x + PX(nr, T, O), y - PY(nr, T, O))
			$$$()
		}
		// 画转子腰
		function $RQ(T, nr = 0, O, style) {
			$$({ color: '#f33', ...style })
			$.moveTo(x + QX(nr, T), y - QY(nr, T)), $.lineTo(x + QX(nr, T, O), y - QY(nr, T, O))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = G, style) {
			for (let nr = 0; nr < NR; nr++) $RP(T, nr, O?.[nr] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, G], style) {
			for (let nr = 0; nr < NR; nr++) $RQ(T, nr, O?.[nr] ?? O?.at?.(-1) ?? O, style)
		}
		// 画间隙密封
		function $RB(T, style) {
			if (!RB) return
			for (let Style = { color: '#00f', ...style }, n = 0; n < N; n++)
				$$(Style), $.Arc(x + PX(n, T), y - PY(n, T), RB, 0, PI2), $$$()
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
		function $RBC(T, n = 0, O = P * 1.1 + RB, style) {
			$$({ color: '#999', ...style })
			let CT = RBC(T, n)[1]
			$.moveTo(x + PX(n, T, O), y - PY(n, T, O))
			$.lineTo(x + PX(n, T), y - PY(n, T))
			$.lineTo(x + PX(n, T) + (O - P) * cos(CT), y - PY(n, T) - (O - P) * sin(CT))
			$$$()
		}
		return Object.assign(
			{ param: $param, x, y, E: $E, BG: $BG, G: $G, GG: $GG },
			{ RP: $RP, RQ: $RQ, PN: $PN, QN: $QN, RB: $RB },
			{ BB: $BB, RR: $RR, SS: $SS, SSS: $SSS, RBC: $RBC }
		)
	}

	// 点集求包络线 dots:[[ [A, R] ]] tt:正向步进、可卷
	function enve(most = min, dots, fix, TT = Tick_, tt = Tick_.keys(), xyk = 1) {
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
			if (t < tickn) M[t] = min(M[t], M[t + 1]) * 0.3125 + max(M[t], M[t + 1]) * 0.6875
		if (fix) for (let t of tt) M[t] = fix(t, TT[t]) ?? M[t]
		M.close(tickn, 0)
		return function* (T = 0, t_ = tt, x = xyk * GX(T), y = xyk * GY(T)) {
			for (let t of t_) yield [x + M[t] * cos(T + TT[t]), y + M[t] * sin(T + TT[t])]
		}
	}
}
