#include "compute_flow/event_filter.h"
namespace event_filter
{
	//medfilt2(varargin)
	void median_filter()
	{
		ROS_INFO_STREAM("median_filter()");
		/*narginchk(1,3);

		[a, mn, padopt] = parse_inputs(varargin{:});
		*/
		parse_inputs();
		/*
		if isempty(a)
		{	
			b = a;
			return
		}

		domain = ones(mn);
		if (rem(prod(mn), 2) == 1)
		{
			tf = hUseIPPL(a, mn);
			*/
			hUseIPPL();
			/*
			if tf
			{
				a = hPadImage(a,mn, padopt);
				*/
				hPadImage();
				/*
				b = medianfiltermex(a, [mn(1) mn(2)]);
			}
			else
			{
				order = (prod(mn)+1)/2;
				b = ordfilt2(a, order, domain, padopt);
			}
		}
		else
		{
			order1 = prod(mn)/2;
			order2 = order1+1;
			b = ordfilt2(a, order1, domain, padopt);
			b2 = ordfilt2(a, order2, domain, padopt);
			if islogical(b)
				b = b | b2;
			else
				b =	imlincomb(0.5, b, 0.5, b2);
			end
		}
		*/
	}

	void parse_inputs()
	{
		ROS_INFO_STREAM("parse_inputs()");
		/*
		a = varargin{1};
		validateattributes(a, ...
			{'uint8','uint16','uint32','int8','int16','int32','single','double','logical'},...
			{'2d','real','nonsparse'}, mfilename, 'A', 1);

		charLocation = [];
		for(k = 2:nargin)
		{
			if (ischar(varargin{k}))
			{
				charLocation = [charLocation k]; %#ok<AGROW>
			}
		}

		if (length(charLocation) > 1)
		{
			error(message('images:medfilt2:tooManyStringInputs'));
		}
		elseif(isempty(charLocation))
		{
			padopt = 'zeros';
		}
		else
		{
			options = {'indexed', 'zeros', 'symmetric'};
			padopt = validatestring(varargin{charLocation}, options, mfilename,'PADOPT', charLocation);
			varargin(charLocation) = [];
		}

		if (strcmp(padopt, 'indexed'))
		{
			if (isa(a,'double'))
			{
				padopt = 'ones';
			}
			else
			{
				padopt = 'zeros';
			}
		}

		if(length(varargin) == 1)
		{
					mn = [3 3];% default
		}
		elseif(length(varargin) >= 2)
		{
			mn = varargin{2}(:)';
			validateattributes(mn,{'numeric'},{'real','positive','integer','nonempty','size',[1 2]},mfilename,'[M N]',2);

			if(length(varargin) > 2)
			{
				error(message('images:medfilt2:invalidSyntax'));	
			}
		*/
	}

	void hUseIPPL()
	{
		ROS_INFO_STREAM("hUseIPPL()");
		/*
		tf = false;
		switch(class(a))
		{
			case 'single':
				if all(mn==[3 3])
				{
					tf = true;
				}
			case 'uint8':
				if (mn(1)==1 && mn(2)<=5) || (all(mn >= [3 3]) && all(mn <= [19 19])) || (mn(2)==1 && mn(1)<=7)
				{
					tf = true;
				}
			case 'uint16' or 'int16':
				if all(mn >= [3 3]) && all(mn <= [19 19])
				{
					tf = true;
				}
		}
			
		tf = tf & iptgetpref('UseIPPL');
		*/
	}

	void hPadImage()
	{
		ROS_INFO_STREAM("hPadImage()");
		/*
		center = floor((domainSize + 1) / 2);
		padSize = domainSize-center;
		if (strcmp(padopt, 'zeros'))
		{
			A = padarray(A, padSize, 0, 'both');
		}
		elseif (strcmp(padopt, 'symmetric'))
		{
		A = padarray(A, padSize, 'symmetric', 'both');
		}
		else
		{
			error(message('images:medfilt2:incorrectPaddingOption'))
		}
		
		*/
	}
}