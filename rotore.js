// Copyright: Qianyan Cai
// License: GPL v3

// 外旋轮线转子引擎 Epitrochoid Rotorary Engine
function RotorE({
	N, // 转子顶角数
	E, // 偏心距
	P = N == 2 ? 1.4 : N == 3 ? 1.9 : 1.1 + N * 0.28, // 转子顶半径 / 偏心距
	RB = 1.18, // 转子缸体间隙 / 顶半径 %
	tickn = 240, // 圆周步进数
	size, // 预估像素
}) {
	if (N != (N |= 0) || N < 2) throw 'err N'
	let NB = N - 1 // 缸体顶数
	let NBS = NB + NB // 缸体冲程数
	let NS = lcm(NBS, 4) // 完整循环冲程数
	let N2 = N + N
	tickn = ceil(tickn / N2 / NB) * N2 * NB // 圆周步进数，转子顶*缸体顶*2 的整倍数

	size = ceil(+size || min(size.width, size.height))
	E = round((E ?? (size * 0.313) / (P + N + 1.75)) * 4) / 4 // 偏心距
	if ((E | 0) < 1) throw 'err E'
	let g = E * NB // 缸体小节圆半径
	let G = E * N // 转子大节圆半径
	P = round(E * (P + N + 2)) // 转子顶半径
	let Q = P - E - E // 转子腰半径
	RB *= P / 100 // 转子缸体间隙

	// 转子、曲轴步进角，均匀
	let Tick_ = (this.Tick_ = [...Array.seq(0, tickn)].map(t => (t / tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, tickn))
	let tPQ = tickn / N2 // 转子顶腰步进
	let TPQ = PI2 / N2 // 转子顶腰夹角
	let tN = n => (n * tickn) / N // 转子腰起始步进
	let TN = n => (n * PI2) / N // 转子腰起始角
	let tS = S => ((S % NBS) * tickn) / NBS // 转子冲程起始步进
	let TS = S => (S * PI2) / NBS // 转子冲程起始角

	// 曲轴心 X=0 Y=0
	let GX = T => E * cos(T * N) // 转子心X
	let GY = T => E * sin(T * N) // 转子心Y
	let PX = (T, n = 0, p = P) => GX(T) + p * cos(T + TN(n) + TPQ) // 转子顶X
	let PY = (T, n = 0, p = P) => GY(T) + p * sin(T + TN(n) + TPQ) // 转子顶Y
	let QX = (T, n = 0, q = Q) => GX(T) + q * cos(T + TN(n)) // 转子腰X
	let QY = (T, n = 0, q = Q) => GY(T) + q * sin(T + TN(n)) // 转子腰X
	let BX = (T, p = P + RB) => PX(T - TPQ, 0, p) // 缸体点X
	let BY = (T, p = P + RB) => PY(T - TPQ, 0, p) // 缸体点Y

	// 缸体型线  E*cos(T*N-PI) + (P+RB)*cos(T), E*cos(T*N-PI) + (P+RB)*cos(T)
	let BB = Tick_.map(T => [BX(T), BY(T)])
	let TB = BB.map(([X, Y]) => atan(X, Y))
	TB[TB.length - 1] = PI2 // 缸体步进角，非均匀

	let S0t = [...Array.seq(-tPQ, tPQ, tickn)] // 冲程0工作线步进，即转子缸体腰线，此处tickn整倍数
	let S1t = [...Array.seq(tS(1) - tPQ, tS(1) + tPQ)] // 冲程1工作线步进，即缸体顶线，此处tickn整倍数
	// 缸体工作线步进，0等同S0t，TS(1)等同S1t，每TS(s)为tickn整倍数
	function St(T, n = 0) {
		let B1 = atan(PX(T, n - 1, P + RB), PY(T, n - 1, P + RB))
		let B = atan(PX(T, n, P + RB), PY(T, n, P + RB))
		return [...Array.seq(round(B1.bfind(TB)) % tickn, round(B.bfind(TB)) % tickn, tickn)] // 近似缸体步进
	}

	// 缸体对转子旋转
	function* BRT(B) {
		if (typeof B == 'number')
			for (let T of Tick_) {
				let X = P * cos(B + T) - E * cos(B * N + T) - E * cos(-T * NB)
				let Y = P * sin(B + T) - E * sin(B * N + T) - E * sin(-T * NB)
				yield [atan(X, Y), dist(X, Y), X, Y] // 角[0,PI2) B==0 沿T严格递增 B>0 沿T循环严格递增
			}
		else for (B of B) yield BRT(B)
	}
	// 转子型线、即缸体绕转子心的内包络线 // RR = MinCurve(RBT(Tick, Tick), true)
	let RR = enve(min, BRT(S0t.map(Tick.At())), t => (t % tPQ ? null : t % (tickn / N) ? P : Q))

	// 工作区型线
	let SS = (T, n = 0) => [...RR(T + TN(n), S0t)].reverse().concat(St(T, n).map(BB.At())).close()
	let V0 = area(SS(0)) // 最小容积
	let VS = (T, n = 0, add0) => area(SS(T, n)) - (add0 ? 0 : V0) // 工作区容积
	let V = VS(TS(1)) // 工作容积
	let K = V / V0 + 1 // 容积比，即压缩比、膨胀比
	let VB = area(BB) // 总体积
	let KB = VB / V // 总体积比工作容积
	let VV = VB - area(RR(0)) // 总容积
	let KK = VV / V // 总容积比工作容积

	// 冲程区型线
	let SSS = [...Array.seq(0, NBS - 1)].map(S => {
		function* RT(T) {
			for (let [X, Y] of RR(T, S0t)) yield [atan(X, Y), dist(X, Y), X, Y]
		}
		function* TT() {
			for (let t of Array.seq(tS(S), tS(S + 1), tickn, true)) yield RT(Tick_[t])
		}
		let st = [...Array.seq(tS(S) - tPQ, tS(S + 1) + tPQ, tickn, true)] // 缸体工作线，此处tickn整倍数
		let s = [...enve(min, TT(), null, TB, st)(0, st, 0, 0)]
		return s.reverse().concat(st.map(BB.At())).close()
	})

	// 转子顶与缸体接触角、及接触步进角
	function RBC(T, n = 0) {
		let CT = atan(PX(T, n) - -g * cos(T * N), PY(T, n) - -g * sin(T * N)) // 两节圆交点--转子顶点
		return [diffabs(T + TN(n) + TPQ - CT), CT]
	}
	let RBCC = max(...Tick.map(T => RBC(T)[0])) // 最大接触角

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { size, N, NS, E, g, G, P, Q, RB, V, K, VV, KK, VB, KB, RBCC })
	Object.assign(this, { TN, TS, BB, RR, SS, VS })

	// 参数显示
	function params(T) {
		let p1 = '__'
		if (T != null) {
			let a = (T / TS(1)) * PI
			let pis = (3 + 1 - sqrt(3 * 3 - sin(a) * sin(a)) - cos(a)) / 2
			p1 = _`|${VS(T) / 100}{03}__${pis}{.2}|${(1 - cos(a)) / 2}{.2}|${VS(T) / V}{.2}__`
		}
		let p2 = T != null ? _`|${(RBC(T)[0] / PI2) * 360}{02}` : ''
		return (
			_`N${N}__E${E}{}__P${P}{}__K${K}{1}__V${V / 100}{}` +
			p1 +
			_`${VV / 100}{}:${KK}{1} ${VB / 100}{}__` +
			_`RB${RB}{1} C${(RBCC / PI2) * 360}{}` +
			p2
		)
	}
	console.log(...params().split('__'), _`Vmin${V0 / 100}{1} tn${tickn}`)

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		let x = midx ?? canvas.width / 2 // 曲轴心X
		let y = midy ?? canvas.height / 2 // 曲轴心Y
		let $param = T => (param.textContent = params(T).replace(/__/g, '\n'))

		function $$({ color = '#000', opa = '', thick = 1 } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			opa.length > 0 && /#....(....)?$/.test(color) && (color = color.slice(0, -opa.length))
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
		}
		function $$$(fill) {
			fill ? $.fill() : $.stroke(), ($.lineWidth = 1)
			fill ? ($.fillStyle = '#000') : ($.strokeStyle = '#000')
		}
		// 画缸体小圆
		function $g(style) {
			$$(style), $.arc(x, y, g, 0, PI2), $$$()
		}
		// 画偏心线，即曲轴
		function $Gg(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo(x, y), $.lineTo(x + GX(T), y + GY(T)), $$$()
		}
		// 画转子大圆
		function $G(T, style) {
			$$({ color: '#999', ...style }), $.arc(x + GX(T), y + GY(T), G, 0, PI2), $$$()
		}
		// 画转子大圆外包
		function $GG(style) {
			$$({ color: '#ddd', ...style }), $.arc(x, y, G + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $P(T, n = 0, O, style) {
			$$({ color: '#00f', ...style })
			RB && $.arc(x + PX(T, n), y + PY(T, n), RB, 0, PI2)
			$.moveTo(x + PX(T, n), y + PY(T, n)), $.lineTo(x + PX(T, n, O), y + PY(T, n, O))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			$$({ color: '#f33', ...style })
			$.moveTo(x + QX(T, n), y + QY(T, n)), $.lineTo(x + QX(T, n, O), y + QY(T, n, O))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = G, style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, G], style) {
			for (let n = 0; n < N; n++) $Q(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子腰旋转线
		function $QQ(style) {
			$$({ color: '#fbb', ...style })
			for (let T of Tick_) (T ? $.lineTo : $.moveTo).call($, x + QX(T), y + QY(T))
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画转子
		function $RR(T, st, style) {
			$$(style)
			let to
			for (let [X, Y] of RR(T, st ? S0t : undefined))
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画工作区
		function $SS(T, n = 0, style) {
			$$(style, true)
			let to
			for (let [X, Y] of SS(T, n)) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$(true)
		}
		// 画冲程区
		function $SSS(S, style) {
			$$(style, true)
			let to
			for (let [X, Y] of SSS[floor(S).mod(NBS)])
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$(true)
		}
		// 画接触角
		function $RBC(T, n = 0, O = P * 1.1 + RB, style) {
			$$({ color: '#999', ...style })
			let CT = RBC(T, n)[1]
			$.moveTo(x + PX(T, n, O), y + PY(T, n, O))
			$.lineTo(x + PX(T, n), y + PY(T, n))
			$.lineTo(x + PX(T, n) + (O - P) * cos(CT), y + PY(T, n) + (O - P) * sin(CT))
			$$$()
		}
		return Object.assign(
			{ param: $param, x, y, g: $g, Gg: $Gg, G: $G, GG: $GG },
			{ P: $P, Q: $Q, PN: $PN, QN: $QN, QQ: $QQ },
			{ BB: $BB, RR: $RR, SS: $SS, SSS: $SSS, RBC: $RBC }
		)
	}

	// 点集求包络线 dots:[[ [A, R] ]] tt:正向步进、可卷
	function enve(most = min, dots, fix, TT = Tick_, tt = Tick_.keys()) {
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
		return function* (T = 0, ttt = tt, x = GX(T), y = GY(T)) {
			for (let t of ttt) yield [x + M[t] * cos(T + TT[t]), y + M[t] * sin(T + TT[t])]
		}
	}
}
