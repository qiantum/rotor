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
	let tQ = n => (n * tickn) / N // 转子腰点起始步进
	let TQ = n => (n * PI2) / N // 转子腰点起始角
	let tS = S => ((S % NBS) * tickn) / NBS // 转子冲程起始步进
	let TS = S => (S * PI2) / NBS // 转子冲程起始角

	// 曲轴心 X=0 Y=0
	let GX = T => E * cos(T * N) // 转子心X
	let GY = T => E * sin(T * N) // 转子心Y
	let PX = (T, n = 0, p = P) => GX(T) + p * cos(T + TQ(n) + TPQ) // 转子顶X
	let PY = (T, n = 0, p = P) => GY(T) + p * sin(T + TQ(n) + TPQ) // 转子顶Y
	let QX = (T, n = 0, q = Q) => GX(T) + q * cos(T + TQ(n)) // 转子腰X
	let QY = (T, n = 0, q = Q) => GY(T) + q * sin(T + TQ(n)) // 转子腰X

	let Qt = [...Array.seq(-tPQ, tPQ, tickn)] // 转子腰线、转子顶对应缸体腰线 步进

	// 缸体型线  E*cos(T*N-PI) + (P+RB)*cos(T), E*cos(T*N-PI) + (P+RB)*cos(T)
	let BB = Tick_.map(T => [PX(T - TPQ, 0, P + RB), PY(T - TPQ, 0, P + RB)])
	let TB = BB.map(([X, Y]) => atan(X, Y))
	TB[TB.length - 1] = PI2 // 缸体步进角，非均匀
	let BPt = [...Array.seq(tS(1) - tPQ, tS(1) + tPQ)] // 缸体顶线步进
	// 缸体冲程线步进
	let BSt = [...Array.seq(0, NBS - 1)].map(S => [
		...Array.seq(tS(S) - tPQ, tS(S + 1) + tPQ, tickn, true),
	])

	let TQB = (T, n = 0) => atan(QX(T, n), QY(T, n)) // 转子腰对应缸体角
	let tQB = (T, n = 0, int = round) => int(TQB(T, n).bfind(TB)) % tickn // 转子腰对应缸体步进
	// 转子顶与缸体接触角、及接触步进角
	function PBC(T, n = 0) {
		let CT = atan(PX(T, n) + g * cos(T * N), PY(T, n) + g * sin(T * N))
		return [CT - T - TQ(n) - TPQ, CT]
	}
	let PBCC = max(...Tick.map(T => PBC(T)[0])) // 最大接触角

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
	// 转子型线、即缸体绕转子心的内包络线
	let RT = minDot(BRT(Qt.map(Tick.At())), t => (t % tPQ ? null : t % (tickn / N) ? P : Q))
	// let RT = MinCurve(RBT(Tick, Tick), true)

	let Vmin = area(Qt.map(BB.At()).concat([...RT(0, Qt)].reverse())) // 最小容积 == VQ(0, 0, true)
	// 转子腰线容积
	function VQ(T, n = 0, withMin) {
		let t = tQB(T, n) // 为0相当于Qt，为tS(1)相当于BPt
		let b = [...Array.seq(t - tPQ, t + tPQ, tickn)].map(BB.At())
		let v = area(b.concat([...RT(T, Qt)].reverse()))
		return round(withMin ? v : v - Vmin)
	}

	let V = VQ(TS(1)) // 工作容积
	let K = V / Vmin + 1 // 容积比，即压缩比、膨胀比
	let VB = area(BB) // 总体积
	let KB = VB / V // 总体积比工作容积
	let VV = VB - area(RT(0)) // 总容积
	let KK = VV / V // 总容积比工作容积

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { size, N, NS, E, G, g, P, Q, RB, V, K, VV, KK, VB, KB, PBCC })
	Object.assign(this, { TPQ, TQ, TS, BB, RT, VQ })

	// 冲程区
	let SS = BSt.map((BSt, S) => {
		function* RR(T) {
			for (let [X, Y] of RT(T, Qt)) yield [atan(X, Y), dist(X, Y), X, Y]
		}
		function* TT() {
			for (let t of Array.seq(tS(S), tS(S + 1), tickn, true)) yield RR(Tick_[t])
		}
		let s = [...minDot(TT(), null, TB, BSt)(0, BSt, 0, 0)]
		s = BSt.map(BB.At()).concat(s.reverse())
		return s.push(s[0]), s
	})

	// 参数显示
	function params(T) {
		let p1 = '__'
		if (T != null) {
			let a = (T / TS(1)) * PI
			let pis = (3 + 1 - sqrt(3 * 3 - sin(a) * sin(a)) - cos(a)) / 2
			p1 = _`|${VQ(T) / 100}{03}__${pis}{.2}|${(1 - cos(a)) / 2}{.2}|${VQ(T) / V}{.2}__`
		}
		let p2 = T != null ? _`|${(diffabs(PBC(T)[0]) / PI2) * 360}{02}` : ''
		return (
			_`N${N}__E${E}{}__P${P}{}__K${K}{1}__V${V / 100}{}` +
			p1 +
			_`${VV / 100}{}:${KK}{1} ${VB / 100}{}__` +
			_`RB${RB}{1} C${(PBCC / PI2) * 360}{}` +
			p2
		)
	}
	console.log(...params().split('__'), _`Vmin${Vmin / 100}{1} tn${tickn}`)

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
			$$({ color: '#0f0', ...style })
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
		// 画转子腰凸包
		function $QQ(style) {
			$$({ color: '#9f9', ...style })
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
		function $RR(T, qt, style) {
			$$(style)
			let to
			for (let [X, Y] of RT(T, qt ? Qt : undefined))
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画转子接触角
		function $PBC(T, n = 0, O = P * 1.1 + RB, style) {
			$$({ color: '#66c', ...style })
			$.moveTo(x + PX(T, n, P + RB), y + PY(T, n, P + RB))
			$.lineTo(x + PX(T, n, O), y + PY(T, n, O))
			let CT = PBC(T, n)[1]
			$.moveTo(x + PX(T, n) + cos(CT), y + PY(T, n) + sin(CT))
			$.lineTo(x + PX(T, n) + (O - P) * cos(CT), y + PY(T, n) + (O - P) * sin(CT))
			$$$()
		}
		// 画冲程区
		function $SS(S, style) {
			$$(style, true)
			let to
			for (let [X, Y] of SS[floor(S).mod(NBS)])
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$(true)
		}
		return Object.assign(
			{ param: $param, x, y, g: $g, Gg: $Gg, G: $G, GG: $GG },
			{ P: $P, Q: $Q, PN: $PN, QN: $QN, QQ: $QQ, BB: $BB, PBC: $PBC, RR: $RR, SS: $SS }
		)
	}

	// 点集求内包络线 dots:[[ [A, R] ]] tt:正向步进、可卷
	function minDot(dots, fix, TT = Tick_, tt = Tick_.keys()) {
		if (TT[0] != 0 || TT[tickn] != PI2) throw 'err TT'
		let M = new Array(tickn).fill(size * 2)
		for (let dot of dots)
			for (let [A, R] of dot) {
				let t = ceil(TT == Tick_ ? (tickn * A) / PI2 : A.bfind(TT)) % tickn
				M[t] = min(M[t], R)
			}
		tt.values ?? (tt = [...tt])
		// M.fillHole(size * 2) 如果tickn太小，部分步进无数据，则需要线性填充
		M[tt[0]] == size * 2 && (M[tt[0]] = M[(tt[0] + 1) % tickn])
		M[(tt.at(-1) + 1) % tickn] == size * 2 && (M[(tt.at(-1) + 1) % tickn] = M[tt.at(-1)])
		M[tickn] = M[0]
		for (let t of tt)
			if (t < tickn) M[t] = min(M[t], M[t + 1]) * 0.3125 + max(M[t], M[t + 1]) * 0.6875
		if (fix) for (let t of tt) M[t] = fix(t, TT[t]) ?? M[t]
		M[tickn] = M[0]

		function* XY(T, ttt = tt, x = GX(T), y = GY(T)) {
			for (let t of ttt) yield [x + M[t] * cos(T + TT[t]), y + M[t] * sin(T + TT[t])]
		}
		return (XY.count = tt.length - 1), XY
	}
}
