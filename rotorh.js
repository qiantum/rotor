// Copyright: Qianyan Cai
// License: GPL v3

// 内旋轮线转子引擎 Hypotrochoid Rotorary Engine
function RotorH({
	NB, // 缸体顶角数
	E, // 偏心距
	P = NB == 2 ? 0.4 : NB == 3 ? 0.9 : 0.1 + NB * 0.28, // 缸体腰半径 / 偏心距
	RB = 1.18, // 转子缸体间隙 / 顶半径 %
	tickn = 240, // 圆周步进数
	size, // 预估像素
}) {
	if (NB != (NB |= 0) || NB < 2) throw 'err N'
	let N = NB - 1 // 转子顶数
	let NRS = N + N // 转子冲程数
	let NS = lcm(NRS, 4) // 完整循环冲程数
	let N2 = N + N
	tickn = ceil(tickn / N2 / NB) * N2 * NB // 圆周步进数，转子顶*缸体顶*2 的整倍数

	size = ceil(+size || min(size.width, size.height))
	E = round((E ?? (size * 0.313) / (P + NB + 3)) * 4) / 4 // 偏心距
	if ((E | 0) < 1) throw 'err E'
	let G = E * NB // 缸体大节圆半径
	let g = E * N // 转子小节圆半径
	P = round(E * (P + NB + 3)) // 转子顶半径
	let Q = P - E - E // 转子腰半径
	RB *= P / 100 // 转子缸体间隙
	let BP = P + E + RB // 缸体顶半径
	let BQ = P - E + RB // 缸体腰半径

	// 缸体、曲轴步进角，均匀
	let Tick_ = (this.Tick_ = [...Array.seq(0, tickn)].map(t => (t / tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, tickn))
	let tPQ = tickn / N2 // 转子顶腰步进
	let TPQ = PI2 / N2 // 转子顶腰夹角
	let tBPQ = tickn / NB / 2 // 缸体顶腰步进
	let TBPQ = PI / NB // 缸体顶腰夹角
	let tN = n => (n * tickn) / N // 转子顶起始步进
	let TN = n => (n * PI2) / N // 转子顶起始角
	let TBN = n => (n * PI2) / NB // 缸体顶起始角
	let tS = S => ((S % NRS) * tickn) / NRS // 转子冲程起始步进
	let TS = S => (S * PI2) / NRS // 转子冲程起始角

	// 曲轴心 X=0 Y=0
	let gX = T => E * cos(-T * N) // 转子心X
	let gY = T => E * sin(-T * N) // 转子心Y
	// 转子点XY，R顶腰处Tick整倍数，顶X==gX(T)+P*cos(T+TN(n)) 顶Y==gY(T)+P*sin(T+TN(n))
	let RX = (T, R, p = P, x = gX(T)) => x + E * cos(R * NB + T) + (p - E) * cos(R + T)
	let RY = (T, R, p = P, y = gY(T)) => y + E * sin(R * NB + T) + (p - E) * sin(R + T)
	let BPX = (n, bp = BP) => bp * cos(TBN(n)) // 缸体顶X，T为缸体自转
	let BPY = (n, bp = BP) => bp * sin(TBN(n)) // 缸体顶Y，T为缸体自转
	let BQX = (n, bq = BQ) => bq * cos(TBN(n) + TBPQ) // 缸体腰X
	let BQY = (n, bq = BQ) => bq * sin(TBN(n) + TBPQ) // 缸体腰Y

	let Pt = [...Array.seq(-tPQ, tPQ, tickn, true)] // 转子顶线步进
	// 转子型线
	function* RR(T, Rt, RT = Tick_) {
		for (let R of Rt?.map(Tick_.At()) ?? RT) yield [RX(T, R), RY(T, R)]
	}
	let TR = Tick_.map(R => atan(RX(0, R, undefined, 0), RY(0, R, undefined, 0)))
	TR[TR.length - 1] = PI2 // 转子步进角，非均匀

	// 转子对缸体旋转
	function* RBT(R, bq = BQ) {
		if (typeof R == 'number')
			for (let T of Tick_) {
				let X = RX(T, R, bq + E)
				let Y = RY(T, R, bq + E)
				yield [atan(X, Y), dist(X, Y), X, Y] // 角[0,PI2) R==0 沿T严格递增 R>0 沿T循环严格递增
			}
		else for (R of R) yield RBT(R, bq)
	}
	// 缸体型线、即转子绕曲轴的外包络线
	let BB = maxDot(RBT(Pt.map(Tick.At())), t => ((t / tBPQ) % 2 == 1 ? BQ : null))
	// 缸体腰包络、即转子绕曲轴的内包络线
	let BQQ = minDot(RBT(Pt.map(Tick.At()), BQ - RB))

	let S0t = [...Array.seq(-tBPQ, tBPQ, tickn)] // 冲程0工作线步进，即缸体顶线，此处tickn整倍数
	let S1t = S0t // 冲程1工作线步进，即缸体顶线，此处tickn整倍数
	// 转子工作线，0等同S0t，TS(1)等同S1t，每TS(s)为tickn整倍数，短于转子顶线
	function St(T, n = 0) {
		let R1 = (atan(BQX(n - 1) - gX(T), BQY(n - 1) - gY(T)) - T).mod(PI2)
		let R = (atan(BQX(n) - gX(T), BQY(n) - gY(T)) - T).mod(PI2)
		return [...Array.seq(round(R1.bfind(TR)) % tickn, round(R.bfind(TR)) % tickn, tickn)] // 近似转子步进
	}

	let SS = (T, n = 0) => [...RR(T, St(T, n))].reverse().concat(St(0, n).map(BB.At())).close() // 工作区型线
	let V0 = area(SS(0)) // 最小容积
	let VS = (T, n = 0, add0) => area(SS(T, n)) - (add0 ? 0 : V0) // 工作区容积
	let V = VS(TS(1)) // 工作容积
	let K = V / V0 + 1 // 容积比，即压缩比、膨胀比
	let VB = area(BB) // 总体积
	let KB = VB / V // 总体积比工作容积
	let VV = VB - area(RR(0)) // 总容积
	let KK = VV / V // 总容积比工作容积

	// 转子冲程线步进
	let BSt = [...Array.seq(0, NRS - 1)].map(S => [
		...Array.seq(tS(S) - tPQ, tS(S + 1) + tPQ, tickn, true),
	])
	// 冲程区型线
	let SSS = BSt.map((BSt, S) => [...RR(0)])

	// 缸体腰与转子接触角、及接触步进角
	function RBC(T, n = 0) {
		let CT = atan(BQX(n) - G * cos(-T * N), BQY(n) - G * sin(-T * N)) // 两节圆交点--缸体腰点
		return [diffabs(TBN(n) + TBPQ - CT), CT]
	}
	let RBCC = max(...Tick.map(T => RBC(T)[0])) // 最大接触角

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { size, NB, N, NS, E, G, g, P, Q, RB, V, K, VV, KK, VB, KB, RBCC })
	Object.assign(this, { TBN, TS, BB, RR, SS, VS })

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
			_`NB${NB}__E${E}{}__P${P}{}__K${K}{1}__V${V / 100}{}` +
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
		// 画缸体大圆
		function $G(style) {
			$$({ color: '#999', ...style }), $.arc(x, y, G, 0, PI2), $$$()
		}
		// 画偏心线，即曲轴
		function $Gg(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo(x, y), $.lineTo(x + gX(T), y + gY(T)), $$$()
		}
		// 画转子小圆
		function $g(T, style) {
			$$(style), $.arc(x + gX(T), y + gY(T), g, 0, PI2), $$$()
		}
		// 画缸体大圆外包
		function $GG(T, style) {
			$$({ color: '#ddd', ...style }), $.arc(x + gX(T), y + gY(T), G + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $P(T, n = 0, O, style) {
			$$({ color: '#00f', ...style })
			let R = TN(n)
			$.moveTo(x + RX(T, R), y + RY(T, R)), $.lineTo(x + RX(T, R, O), y + RY(T, R, O))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = [0, g], style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画缸体全部腰
		function $BQN(style) {
			if (!RB) return
			style = { color: '#f00', ...style }
			for (let n = 0; n < NB; n++) $$(style), $.arc(x + BQX(n), y + BQY(n), RB, 0, PI2), $$$()
		}
		// 画缸体腰包络
		function $BQQ(style) {
			$$({ color: '#fbb', ...style })
			let to
			for (let [X, Y] of BQQ) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
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
			for (let [X, Y] of RR(T, st ? St(T) : undefined))
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
			for (let [X, Y] of SSS[floor(S).mod(NRS)])
				(to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$(true)
		}
		// 画接触角
		function $RBC(T, n = 0, O = BQ * 0.9 - RB, style) {
			$$({ color: '#999', ...style })
			let CT = RBC(T, n)[1]
			$.moveTo(x + BQX(n, O), y + BQY(n, O))
			$.lineTo(x + BQX(n), y + BQY(n))
			$.lineTo(x + BQX(n) + (O - BQ) * cos(CT), y + BQY(n) + (O - BQ) * sin(CT))
			$$$()
		}
		return Object.assign(
			{ param: $param, x, y, G: $G, Gg: $Gg, g: $g, GG: $GG },
			{ P: $P, PN: $PN, BQN: $BQN, BQQ: $BQQ },
			{ BB: $BB, RR: $RR, SS: $SS, SSS: $SSS, RBC: $RBC }
		)
	}

	// 点集求外包络线 dots:[[ [A, R] ]] tt:正向步进、可卷
	function maxDot(dots, fix, TT = Tick_, tt = Tick_.keys()) {
		if (TT[0] != 0 || TT[tickn] != PI2) throw 'err TT'
		let M = new Array(tickn).fill(0)
		for (let dot of dots)
			for (let [A, R] of dot) {
				let t = ceil(TT == Tick_ ? (tickn * A) / PI2 : A.bfind(TT)) % tickn
				M[t] = max(M[t], R)
			}
		tt.values ?? (tt = [...tt])
		// M.fillHole(0) 如果tickn太小，部分步进无数据，则需要线性填充
		M[tt[0]] == 0 && (M[tt[0]] = M[(tt[0] + 1) % tickn])
		M[(tt.at(-1) + 1) % tickn] == 0 && (M[(tt.at(-1) + 1) % tickn] = M[tt.at(-1)])
		M.close(tickn, 0)
		for (let t of tt)
			if (t < tickn) M[t] = max(M[t], M[t + 1]) * 0.3125 + min(M[t], M[t + 1]) * 0.6875
		if (fix) for (let t of tt) M[t] = fix(t, TT[t]) ?? M[t]
		M.close(tickn, 0)
		return tt.map(t => [M[t] * cos(TT[t]), M[t] * sin(TT[t])])
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
		M.close(tickn, 0)
		for (let t of tt)
			if (t < tickn) M[t] = max(M[t], M[t + 1]) * 0.3125 + min(M[t], M[t + 1]) * 0.6875
		if (fix) for (let t of tt) M[t] = fix(t, TT[t]) ?? M[t]
		M.close(tickn, 0)
		return tt.map(t => [M[t] * cos(TT[t]), M[t] * sin(TT[t])])
	}
}
