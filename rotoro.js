let { min, max, abs, floor, ceil, round, PI, cos, sin, sqrt } = Math
let PI2 = (Math.PI2 = PI + PI)
let EPSI = (Number.EPSILON = 1 / (1 << 12))
let atan, diff, diffAbs, cross, area

function Rotor({
	N, // 转角数
	E, // 偏心距
	P = (N - 1) * 0.8, // 转子角半径 / 偏心距
	Q = P, // 转子腰半径 / 偏心距
	BP = 0.1, // 缸体转子间隙 / 偏心距
	Tick: Tickn = 16, // 每角旋转步进
	Rfast, // 快速计算转子型线
	size, // 预估像素
}) {
	let PIN = PI / N
	let N2 = N + N
	Tickn = floor(min(Tickn * N2, 160) / N2) * N2 // 每角旋转步进
	size = ceil(+size || min(size.clientWidth, size.clientHeight))
	E ??= floor(size / (2.3 * (P + N + 3))) // 偏心距
	let G = E * N // 转子大节圆半径
	let g = G - E // 曲轴小节圆半径
	P = E * (P + N + 2) // 转子角半径
	Q = E * (Q + N) // 转子腰半径
	BP *= E // 缸体转子间隙

	let Tn = n => (n * PI2) / N
	let Tst = st => (st * PI) / (N - 1)
	let tT = (T, int = round) => int((Tickn * T) / PI2) % Tickn // [0,Tickn)
	let Tick_ = (this.Tick_ = [...Array.seq(0, Tickn)].map(t => (t / Tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, Tickn))

	// 缸体型线
	let BB = Tick_.map(T => [
		E * cos(T * N - PI) + (P + BP) * cos(T),
		E * sin(T * N - PI) + (P + BP) * sin(T),
	])
	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	// 缸体腰线
	let BQt = [...Array.seq(Tickn - Tickn / N2, Tickn - 1), ...Array.seq(0, Tickn / N2)]
	// 缸体顶线
	let TBP = Tst(1)
	let BPt = [...Array.seq(Tick.binFind(TBP - PIN), Tick.binFind(TBP + PIN))]

	// 转子型线、即缸体绕转子心的内包络线
	let R, RR
	if (Rfast) {
		RR = new Array(Tickn).fill(P)
		for (let T of Tick)
			for (let B of BQt.map(t => Tick[t])) {
				let X = P * cos(B + T) - E * cos(B * N + T) - E * cos(T * (N - 1))
				let Y = P * sin(B + T) - E * sin(B * N + T) + E * sin(T * (N - 1))
				let t = tT(atan(X, Y), ceil)
				RR[t] = min(RR[t], sqrt(X * X + Y * Y))
			}
		RR.push(Q)
		for (let t = 0; t < Tickn; t++)
			if (t % (Tickn / N2) == 0) RR[t] = t % (Tickn / N) == 0 ? Q : P
			else RR[t] = min(RR[t], RR[t + 1]) * 0.375 + max(RR[t], RR[t + 1]) * 0.625

		R = function* (T, rt = Array.seq(0, Tickn)) {
			let x = E * cos(T * N)
			let y = E * sin(T * N)
			for (let t of rt) yield [x + RR[t] * cos(T + Tick_[t]), y + RR[t] * sin(T + Tick_[t])]
		}
	} else {
		for (let B of Tick) {
			// 缸体转动
			let bb = Tick.map(T => {
				let X = P * cos(B + T) - E * cos(B * N + T) - E * cos(T * (N - 1))
				let Y = P * sin(B + T) - E * sin(B * N + T) + E * sin(T * (N - 1))
				let A = atan(X, Y) // [0,PI2) C==0 沿T严格递增 [0,PI2] C>0 沿T循环严格递增
				return [A, sqrt(X * X + Y * Y), X, Y, RR?.binFind(A, 0, -1)]
			})
			if (RR != (RR ??= [...bb])) continue
			let S = []
			// 对缸体每段
			for (let bt = 0; bt < Tickn; bt++) {
				let [a_, r_, x_, y_, t_] = bb[bt]
				let [a, r, x, y, t] = bb[bt + 1] || bb[0]
				t_--, (t %= RR.length) // 对应转子连续段
				for (let T_ = t_, T1, T; T_ != t; T_ = T) {
					let [A_, R_, X_, Y_] = RR[T_]
					let [A, R, X, Y] = RR[(T = T1 = T_ + 1)] || RR[(T = 0)]
					// 求交点和半径
					let [SX, SY, abc, abd, cda, cdb] = cross(X_, Y_, X, Y, x_, y_, x, y)
					if (T != t && cdb < -EPSI && T) S.push([1 / 0, 0, 0, 0, T]) // 去掉长半径转子点
					if (T_ == t_ && abc > EPSI) S.push([a_, r_, x_, y_, T1]) // 短半径缸体点
					if (T == t && abd > EPSI) S.push([a, r, x, y, T1]) // 短半径缸体点
					if (SX != null) S.push([atan(SX, SY), sqrt(SX * SX + SY * SY), SX, SY, T1]) //交点
				}
			}
			S.sort((s1, s2) => s1[4] - s2[4] || s1[0] - s2[0]) // 保持角度有序
			let t = 0 // 加减点
			for (let s of S)
				if (s[0] == 1 / 0) RR.splice(s[4] + t--, 1)
				else RR.splice(s[4] + t++, 0, s), s.length--
			RR[RR.push([...RR[0]]) - 1][0] += PI2 // 闭合
			t = 0 // 合并重合点
			for (let R of RR)
				if (R[0] < RR[t][0]) throw 'err a'
				else if (R[0] - RR[t][0] > EPSI && abs(R[1] - RR[t][1]) > EPSI) RR[++t] = R
			RR.length = ++t
		}

		R = function* (T, rt = Array.seq(0, Tickn)) {
			let s = rt.values?.() ?? rt
			let tt = s.next().value
			if (tt == null) return
			let x = E * cos(T * N)
			let y = E * sin(T * N)
			let r = RR.binFind(Tick_[tt], 0)
			for (let t of s)
				do
					for (let A, tT = Tick_[t] + (tt > t && PI2); (A = RR[r]?.[0]) <= tT; r++)
						yield [x + RR[r][1] * cos(T + A), y + RR[r][1] * sin(T + A)]
				while (tt > (tt = t) && !(r = 0))
		}
	}
	console._`R in ${RR.length - 1} details`

	// 工作容积，总容积，总体积
	let V, K, VV, KK, VB, KB
	{
		let v = area(BQt.map(t => BB[t])) - area(R(0, BQt))
		V = area(BPt.map(t => BB[t])) - area(R(TBP, BQt)) - v
		VB = area(BB)
		VV = VB - area(R(0))
		;(K = V / v), (KK = VV / V), (KB = VB / V)
		console._`Vmin ${v}{} Vmax ${V}{} K ${K}{1}  KK ${KK}{1} KB ${KB}{1}`
	}

	Object.assign(this, { N, E, G, g, P, Q, BP, R, V, K, KK, KB, Tn, Tst, size })

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		if (param)
			param.textContent =
				_`N${N}{} E${E}{}\nP${P / E}{1} Q${Q / E}{1}\n` + _`K${K}{} ${KK}{1} ${KB}{1}`

		let $x = midx ?? canvas.clientWidth / 2
		let $y = midy ?? canvas.clientHeight / 2
		let $X = T => $x + E * cos(T * N) // 转子心X
		let $Y = T => $y + E * sin(T * N) // 转子心Y

		function $$({ color = '#000', opa = '', thick = 1 } = {}) {
			$.beginPath(), ($.strokeStyle = color + opa), ($.lineWidth = thick)
		}
		function $$$() {
			$.stroke(), ($.strokeStyle = '#000'), ($.lineWidth = 1)
		}
		// 画曲轴小圆
		function $g(style) {
			$$(style), $.arc($x, $y, g, 0, PI2), $$$()
		}
		// 画偏心线
		function $Gg(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo($x, $y), $.lineTo($X(T), $Y(T)), $$$()
		}
		// 画转子大圆
		function $G(T, style) {
			$$({ color: '#999', ...style }), $.arc($X(T), $Y(T), G, 0, PI2), $$$()
		}
		// 画转子大圆凸包
		function $GG(style) {
			$$({ color: '#999', opa: '5', ...style }), $.arc($x, $y, G + E, 0, PI2), $$$()
		}
		// 画转子角
		function $P(T, n = 0, O, style) {
			T += Tn(n)
			$$({ color: '#00f', ...style })
			let X = $X(T) + P * cos(T + PIN)
			let Y = $Y(T) + P * sin(T + PIN)
			BP && $.arc(X, Y, BP, 0, PI2)
			$.moveTo(X, Y), $.lineTo($X(T) + O * cos(T + PIN), $Y(T) + O * sin(T + PIN))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			T += Tn(n)
			$$({ color: '#0f0', ...style })
			$.moveTo($X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			$.lineTo($X(T) + O * cos(T), $Y(T) + O * sin(T))
			$$$()
		}
		// 画转子全部角
		function $PN(T, O = G, style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.[O?.length - 1] ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, G], style) {
			for (let n = 0; n < N; n++) $Q(T, n, O?.[n] ?? O?.[O?.length - 1] ?? O, style)
		}
		// 画转子腰凸包
		function $QQ(style) {
			$$({ color: '#9f9', ...style })
			for (let T of Tick_) {
				let to = T ? $.lineTo : $.moveTo
				to.call($, $X(T) + Q * cos(T), $Y(T) + Q * sin(T))
			}
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		// 画转子
		function $RR(T, style) {
			$$(style)
			let to
			for (let [X, Y] of R(T)) (to = to ? $.lineTo : $.moveTo).call($, $x + X, $y + Y)
			$$$()
		}
		return {
			x: $x,
			y: $y,
			g: $g,
			Gg: $Gg,
			G: $G,
			GG: $GG,
			P: $P,
			Q: $Q,
			PN: $PN,
			QN: $QN,
			QQ: $QQ,
			BB: $BB,
			RR: $RR,
		}
	}
}

atan = function (x, y) {
	if (abs(x) < EPSI) return y > 0 ? PI * 0.5 : PI * 1.5
	return Math.atan(y / x) + (x < 0 ? PI : y < 0 ? PI2 : 0)
}
diff = v => ((v %= PI2) > PI ? v - PI2 : v < -PI ? v + PI2 : v)
diffAbs = v => ((v = abs(v) % PI2) > PI ? PI2 - v : v)

Array.seq = function* (from, to) {
	for (let i = from; i <= to; i++) yield i
}
Array.prototype.binFind = function (v, prop, epsi = EPSI, neg) {
	let l = 0
	for (let h = this.length - 1, m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = v - (prop != null ? this[m][prop] : this[m]))
		if (c >= -epsi && c <= epsi) return m
		c < 0 ? (h = m - 1) : (l = m + 1)
	}
	return neg ? ~l : l
}
cross = function (ax, ay, bx, by, cx, cy, dx, dy, co) {
	let abc = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
	let abd = (ax - dx) * (by - dy) - (ay - dy) * (bx - dx)
	let cda = (cx - ax) * (dy - ay) - (cy - ay) * (dx - ax)
	let cdb = abc - abd + cda
	if (abc * abd >= 0 || cda * cdb >= 0) {
		if (!co || (abc && abd)) return [null, null, abc, abd, cda, cdb]
		return cda == 0 ? [ax, ay, abc, abd, cda, cdb] : [bx, by, abc, abd, cda, cdb]
	}
	let t = cda / (abd - abc)
	return [ax + t * (bx - ax), ay + t * (by - ay), abc, abd, cda, cdb]
}
area = function (s) {
	let [x0, y0] = (s = s.values?.() ?? s).next().value
	let [xx, yy] = [x0, y0]
	let a = 0
	for (let [x, y] of s) (a += (xx - x) * (yy + y)), (xx = x), (yy = y)
	return (a + (xx - x0) * (yy + y0)) / 2
}
